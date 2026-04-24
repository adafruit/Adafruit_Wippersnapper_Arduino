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
    WS_DEBUG_PRINTLN("[expander] ERROR: Unable to decode expander B2D envelope");
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
 * @brief Handles an expander Add message.
 * @param msg The Add message.
 * @return True if the expander was added successfully, False otherwise.
 */
bool ExpanderController::Handle_Add(ws_expander_Add *msg) {
  if (!msg->has_cfg) {
    WS_DEBUG_PRINTLN("[expander] ERROR: No I2C config provided in Add!");
    return false;
  }

  // TODO: Implement expander hardware initialization
  WS_DEBUG_PRINTLN("[expander] Add handler stub - hardware init not yet "
                   "implemented");

  // Build and publish the Added response
  ws_i2c_DeviceAddedOrReplaced response;
  memset(&response, 0, sizeof(response));
  response.has_device_description = true;
  response.device_description = msg->cfg.device_description;
  response.bus_status = ws_i2c_BusStatus_BS_UNSPECIFIED;
  response.device_status = ws_i2c_DeviceStatus_DS_UNSPECIFIED;

  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_expander_tag,
                     _model->GetAddedD2B(response))) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Failed to publish Added response!");
    return false;
  }
  return true;
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

  // TODO: Implement expander hardware removal
  WS_DEBUG_PRINTLN("[expander] Remove handler stub - hardware removal not yet "
                   "implemented");

  // Build and publish the Removed response
  ws_i2c_DeviceRemoved response;
  memset(&response, 0, sizeof(response));
  response.has_device_description = true;
  response.device_description = msg->cfg.device_description;
  response.did_remove = false;

  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_expander_tag,
                     _model->GetRemovedD2B(response))) {
    WS_DEBUG_PRINTLN("[expander] ERROR: Failed to publish Removed response!");
    return false;
  }
  return true;
}
