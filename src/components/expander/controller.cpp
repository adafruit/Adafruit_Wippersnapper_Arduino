/*!
 * @file src/components/expander/controller.cpp
 *
 * Controller for WipperSnapper's expander component, bridges between the
 * expander.proto API, the model, and the hardware layer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

ExpanderController::ExpanderController() { _model = new ExpanderModel(); }

ExpanderController::~ExpanderController() {
  if (_model) {
    delete _model;
    _model = nullptr;
  }
  for (ExpanderHardware *drv : _expanders) {
    delete drv;
  }
  _expanders.clear();
}

/*!
 * @brief Routes messages using the expander.proto API to the
 *        appropriate controller functions.
 * @param stream The nanopb input stream.
 * @return True if the message was successfully routed, False otherwise.
 */
bool ExpanderController::Router(pb_istream_t *stream) {
  ws_expander_B2D b2d = ws_expander_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_expander_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN(
        "[expander] ERROR: Unable to decode expander B2D envelope");
    return false;
  }

  bool res = false;
  switch (b2d.which_payload) {
  case ws_expander_B2D_add_tag:
    res = Handle_Add(&b2d.payload.add);
    break;
  case ws_expander_B2D_remove_tag:
    res = Handle_Remove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[expander] WARNING: Unsupported expander payload");
    res = false;
    break;
  }
  return res;
}

/*!
 * @brief Adds an I2C expander hardware instance to the controller.
 * @param device_name Name of the expander device (e.g., "mcp23017").
 * @param i2c_addr I2C address of the expander.
 * @param wire Pointer to the TwoWire instance for I2C communication.
 * @return True if the expander was added successfully, false otherwise.
 */
bool ExpanderController::AddExpander(const char *device_name, uint8_t i2c_addr,
                                     TwoWire *wire) {
  // Create the appropriate driver for the expander
  ExpanderHardware *drv = nullptr;
  if (strcmp(device_name, "mcp23017") == 0) {
    drv = new ExpanderMCP23X17();
  } else if (strcmp(device_name, "mcp23008") == 0) {
    drv = new ExpanderMCP23X08();
  } else if (strcmp(device_name, "aw9523") == 0) {
    drv = new ExpanderAW9523();
  } else if (strcmp(device_name, "pcf8574") == 0) {
    drv = new ExpanderPCF8574();
  } else if (strcmp(device_name, "pcf8575") == 0) {
    drv = new ExpanderPCF8575();
  } else if (strcmp(device_name, "tca8418") == 0) {
    drv = new ExpanderTCA8418();
  } else if (strcmp(device_name, "seesaw") == 0) {
    drv = new ExpanderSeesaw();
  } else if (strcmp(device_name, "ads1015") == 0) {
    drv = new ExpanderADS1015();
  } else if (strcmp(device_name, "ads1115") == 0) {
    drv = new ExpanderADS1115();
  } else {
    WS_DEBUG_PRINTLN("[expander] ERROR: Unsupported expander device type!");
    return false;
  }

  if (drv == nullptr) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Unsupported expander device!");
    return false;
  }

  if (!drv->begin(i2c_addr, wire)) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Failed to initialize expander!");
    delete drv;
    return false;
  }

  _expanders.push_back(drv);
  WS_DEBUG_PRINTLN("[expander] Expander hardware added successfully!");
  return true;
}

/*!
 * @brief Finds an expander by its I2C address.
 * @param addr The I2C address to search for.
 * @return Pointer to the ExpanderHardware, or nullptr if not found.
 */
ExpanderHardware *ExpanderController::GetDriver(uint8_t addr) {
  for (ExpanderHardware *drv : _expanders) {
    if (drv->getAddress() == addr)
      return drv;
  }
  return nullptr;
}

/*!
 * @brief Handles an expander Add message.
 * @param msg The Add message.
 * @return True if the expander was added successfully, False otherwise.
 */
bool ExpanderController::Handle_Add(ws_expander_Add *msg) {
  if (!msg->has_cfg) {
    WS_DEBUG_PRINTLN("[expander] ERROR: No configuration provided!");
    return false;
  }

  ws_i2c_DeviceDescriptor desc = msg->cfg.device_description;
  uint8_t addr = (uint8_t)desc.device_address;

  // Check if this expander has already been added
  if (GetDriver(addr) != nullptr) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Expander exists at this address!");
    return false;
  }

  // Get or create the I2C bus for the expander
  TwoWire *wire =
      Ws._i2c_controller->GetOrCreateI2cBus(desc.pin_scl, desc.pin_sda);
  if (wire == nullptr) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Failed to get/create I2C bus!");
    return false;
  }

  // Attempt to initialize the expander
  bool did_add = AddExpander(msg->cfg.device_name, addr, wire);

  // Build and publish the Added response
  ws_i2c_DeviceAddedOrReplaced response;
  memset(&response, 0, sizeof(response));
  response.has_device_description = true;
  response.device_description = desc;
  response.bus_status = ws_i2c_BusStatus_BS_SUCCESS;
  response.device_status = did_add ? ws_i2c_DeviceStatus_DS_SUCCESS
                                   : ws_i2c_DeviceStatus_DS_FAIL_INIT;

  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_expander_tag,
                     _model->GetAddedD2B(response))) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Failed to publish Added response!");
    return false;
  }
  return did_add;
}

/*!
 * @brief Handles an expander Remove message.
 * @param msg The Remove message.
 * @return True if the expander was removed successfully, False otherwise.
 */
bool ExpanderController::Handle_Remove(ws_expander_Remove *msg) {
  if (!msg->has_cfg) {
    WS_DEBUG_PRINTLN("[expander] ERROR: No I2C config provided in Remove!");
    return false;
  }

  ws_i2c_DeviceDescriptor desc = msg->cfg.device_description;
  uint8_t addr = (uint8_t)desc.device_address;
  bool did_remove = false;

  // Find and remove the expander by address
  for (size_t i = 0; i < _expanders.size(); i++) {
    if (_expanders[i]->getAddress() == addr) {
      delete _expanders[i];
      _expanders.erase(_expanders.begin() + i);
      did_remove = true;
      break;
    }
  }

  if (!did_remove) {
    WS_DEBUG_PRINTLN("[expander] WARNING: Expander not found for removal!");
  }

  // Build and publish the Removed response
  ws_i2c_DeviceRemoved response;
  memset(&response, 0, sizeof(response));
  response.has_device_description = true;
  response.device_description = desc;
  response.did_remove = did_remove;

  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_expander_tag,
                     _model->GetRemovedD2B(response))) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Failed to publish Removed response!");
    return false;
  }
  return did_remove;
}
