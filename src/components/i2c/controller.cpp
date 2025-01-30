/*!
 * @file controller.cpp
 *
 * Controller for the i2c.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

// lambda function to create drvBase driver
using FnCreateI2CDriver =
    std::function<drvBase *(TwoWire *, uint16_t, uint32_t, const char *)>;

// Map of sensor names to lambda functions that create an I2C device driver
// NOTE: This list is NOT comprehensive, it's a  subset for now
// to assess the feasibility of this approach.
// TODO: Implement as hash table, unsorted_map instead
static std::map<std::string, FnCreateI2CDriver> I2cFactory = {
    {"bme280",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBme280(i2c, addr, mux_channel, driver_name);
     }}};

drvBase *createI2CDriverByName(const char *driver_name, TwoWire *i2c,
                               uint16_t addr, uint32_t i2c_mux_channel,
                               wippersnapper_i2c_I2cDeviceStatus &status) {
  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
  auto it = I2cFactory.find(driver_name);
  if (it == I2cFactory.end()) {
    status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }

  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;
  // Call the lambda to create the driver
  return it->second(i2c, addr, i2c_mux_channel, driver_name);
}

/***********************************************************************/
/*!
    @brief  I2cController constructor
*/
/***********************************************************************/
I2cController::I2cController() {
  _i2c_bus_alt = nullptr;
  _i2c_model = new I2cModel();
  // Initialize a default I2C bus
  _i2c_bus_default = new I2cHardware();
  _i2c_bus_default->InitBus(true);
}

/***********************************************************************/
/*!
    @brief  I2cController destructor
*/
/***********************************************************************/
I2cController::~I2cController() {
  if (_i2c_model)
    delete _i2c_model;

  if (_i2c_bus_default)
    delete _i2c_bus_default;
}

/*************************************************************************/
/*!
    @brief    Returns the status of the I2C bus
    @param    is_alt_bus
              True if the alt. I2C bus is being queried, False otherwise.
    @returns  True if the I2C bus is operational, False otherwise.
*/
/*************************************************************************/
bool I2cController::IsBusStatusOK(bool is_alt_bus) {
  if (is_alt_bus) {
    return (_i2c_bus_alt->GetBusStatus() ==
            wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS);
  }

  return (_i2c_bus_default->GetBusStatus() ==
          wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS);
}

/***********************************************************************/
/*!
    @brief    Publishes an I2cDeviceAddedorReplaced message to the broker
    @param    device_descriptor
                The I2cDeviceDescriptor message.
    @param    device_status
                The I2cDeviceStatus message.
    @returns  True if the I2cDeviceAddedorReplaced message was published
              successfully, False otherwise.
*/
/***********************************************************************/
bool I2cController::PublishI2cDeviceAddedorReplaced(
    const wippersnapper_i2c_I2cDeviceDescriptor &device_descriptor,
    const wippersnapper_i2c_I2cDeviceStatus &device_status) {
  if (!_i2c_model->encodeMsgI2cDeviceAddedorReplaced(
          device_descriptor, _i2c_bus_default->GetBusStatus(), device_status)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to encode I2cDeviceAddedorReplaced message!");
    return false;
  }

  // TODO: Remove in PR, debug information only!
  WS_DEBUG_PRINTLN("[i2c] I2cDeviceAddedorReplaced Message Contents:");
  WS_DEBUG_PRINT("\t Bus Status: ")
  WS_DEBUG_PRINTLN(
      _i2c_model->GetMsgI2cDeviceAddedOrReplaced()->i2c_bus_status);
  WS_DEBUG_PRINT("\t Device Status: ")
  WS_DEBUG_PRINTLN(
      _i2c_model->GetMsgI2cDeviceAddedOrReplaced()->i2c_device_status);

  if (WsV2._sdCardV2->isModeOffline())
    return true; // Back out if we're in offline mode

  if (!WsV2.PublishSignal(
          wippersnapper_signal_DeviceToBroker_i2c_device_added_replaced_tag,
          _i2c_model->GetMsgI2cDeviceAddedOrReplaced())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish I2cDeviceAddedorReplaced "
                     "message to IO!");
    return false;
  }
  return true;
}

/***********************************************************************/
/*!
    @brief    Implements handling for a I2cDeviceRemove message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceRemove message was handled, False
              otherwise.
*/
/***********************************************************************/
bool I2cController::Handle_I2cDeviceRemove(pb_istream_t *stream) {
  // Attempt to decode an I2cDeviceRemove message
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceRemove message...");
  if (!_i2c_model->DecodeI2cDeviceRemove(stream)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2cDeviceRemove message!");
    return false;
  }

  // TODO: Implement the rest of this function
  WS_DEBUG_PRINTLN("[i2c] I2cDeviceRemove message not yet implemented!");

  return true;
}

/***********************************************************************/
/*!
    @brief    Implements handling for a I2cDeviceAddOrReplace message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceAddOrReplace message was handled
              (created or replaced), False otherwise.
*/
/***********************************************************************/
bool I2cController::Handle_I2cDeviceAddOrReplace(pb_istream_t *stream) {
  bool use_alt_bus = false;
  bool did_set_mux_ch = false;
  // Attempt to decode an I2cDeviceAddOrReplace message
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceAddOrReplace message...");
  if (!_i2c_model->DecodeI2cDeviceAddReplace(stream)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to decode I2cDeviceAddOrReplace message!");
    return false;
  }
  wippersnapper_i2c_I2cDeviceStatus device_status =
      wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_UNSPECIFIED;
  wippersnapper_i2c_I2cDeviceDescriptor device_descriptor =
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_description;
  char device_name[15];
  strcpy(device_name,
         _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name);

  // TODO: Handle Replace messages by implementing a Remove handler first...then
  // proceed to adding a new device

  // Does the device's descriptor specify a different i2c bus?
  if (strcmp(device_descriptor.i2c_bus_scl, "default") != 0) {
    WS_DEBUG_PRINTLN("[i2c] Non-default I2C bus specified!");
    if (_i2c_bus_alt == nullptr) {
      WS_DEBUG_PRINT("[i2c] Initializing alternative i2c bus...");
      _i2c_bus_alt = new I2cHardware();
      _i2c_bus_alt->InitBus(false, device_descriptor.i2c_bus_sda,
                            device_descriptor.i2c_bus_scl);
    }
    use_alt_bus = true;
  }

  // Before we do anything on the bus - was the bus initialized correctly?
  if (!IsBusStatusOK(use_alt_bus)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C bus is not operational or stuck, please "
                     "restart device!");
    if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
      return false;
    return true;
  }

  // Mux case #1 - We are creating a mux via I2cDeviceAddorReplace message
  // TODO: Refactor
  if ((strcmp(device_name, "pca9546") == 0)) {
    WS_DEBUG_PRINTLN("[i2c] Creating a new MUX driver obj");
    if (use_alt_bus) {
      if (!_i2c_bus_alt->HasMux()) {
        _i2c_bus_alt->AddMuxToBus(device_descriptor.i2c_mux_address,
                                  device_name);
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Mux specified but not created");
      }
    } else {
      if (!_i2c_bus_default->HasMux()) {
        WS_DEBUG_PRINT("[i2c] Adding MUX to default bus...");
        _i2c_bus_default->AddMuxToBus(device_descriptor.i2c_mux_address,
                                      device_name);
        WS_DEBUG_PRINTLN("added!");
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Mux specified but not created");
      }
    }
    // TODO: Publish back out to IO instead of blindly returning true
    return true;
  }

  // Mux case #2 - We are creating a new driver that USES THE MUX via
  // I2cDeviceAddorReplace message
  if (device_descriptor.i2c_mux_address != 0x00) {
    // TODO: Remove all debug prints for PR build
    WS_DEBUG_PRINTLN("[i2c] Device requests a MUX channel!");
    uint32_t mux_channel = device_descriptor.i2c_mux_channel;
    if (use_alt_bus) {
      if (_i2c_bus_alt->HasMux()) {
        WS_DEBUG_PRINT("[i2c] Selecting MUX ch# ");
        WS_DEBUG_PRINTLN(mux_channel);
        _i2c_bus_alt->SelectMuxChannel(mux_channel);
        WS_DEBUG_PRINTLN("[i2c] MUX channel selected!");
        did_set_mux_ch = true;
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Device requests a MUX but MUX has not "
                         "been initialized first");
        device_status =
            wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
        PublishI2cDeviceAddedorReplaced(device_descriptor, device_status);
        return false;
      }
    } else {
      if (_i2c_bus_default->HasMux()) {
        WS_DEBUG_PRINT("[i2c] Selecting MUX ch# ");
        WS_DEBUG_PRINTLN(mux_channel);
        _i2c_bus_default->SelectMuxChannel(mux_channel);
        WS_DEBUG_PRINTLN("[i2c] MUX channel selected!");
        did_set_mux_ch = true;
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Device requests a MUX but MUX has not "
                         "been initialized first");
        device_status =
            wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
        PublishI2cDeviceAddedorReplaced(device_descriptor, device_status);
        return false;
      }
    }
  }

  WS_DEBUG_PRINTLN("Creating a new I2C driver obj");
  TwoWire *bus = nullptr;
  if (use_alt_bus) {
    bus = _i2c_bus_alt->GetBus();
  } else {
    bus = _i2c_bus_default->GetBus();
  }

  drvBase *drv = createI2CDriverByName(
      device_name, bus, device_descriptor.i2c_device_address,
      device_descriptor.i2c_mux_channel, device_status);

  // TODO: Clean up for clarity - confusing checks and returns
  if (drv != nullptr) {
    WS_DEBUG_PRINTLN("OK! Configuring sensor types...");
    drv->EnableSensorReads(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_sensor_types,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()
            ->i2c_device_sensor_types_count);
    if (drv->begin()) {
      _i2c_drivers.push_back(drv);
      WS_DEBUG_PRINTLN("[i2c] I2C driver added to controller: ");
      WS_DEBUG_PRINTLN(device_name);

      if (use_alt_bus)
        drv->EnableAltI2CBus();

      // If we're using a MUX, clear the channel for any subsequent bus
      // operations that may not involve the MUX
      if (did_set_mux_ch) {
        if (use_alt_bus) {
          _i2c_bus_alt->ClearMuxChannel();
        } else {
          _i2c_bus_default->ClearMuxChannel();
        }
      }
    } else {
      WS_DEBUG_PRINTLN("[i2c] ERROR: I2C driver failed to initialize!");
      device_status =
          wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
      if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
        return false;
      return true;
    }
  } else {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C driver type not found or unsupported!");
    device_status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
      return false;
    return true;
  }

  // Create and publish the I2cDeviceAddedorReplaced message to the broker
  if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
    return false;
  return true;
}

void I2cController::update() {
  if (_i2c_drivers.size() == 0)
    return; // bail out if no drivers exist

  for (auto *drv : _i2c_drivers) {
    // Does this driver have any enabled sensors?
    size_t sensor_count = drv->GetEnabledSensorCnt();
    if (sensor_count == 0)
      continue; // bail out if driver has no sensors enabled
    // Did driver's period elapse yet?
    ulong cur_time = millis();
    if (cur_time - drv->GetSensorPeriodPrv() < drv->GetSensorPeriod())
      continue; // bail out if the period hasn't elapsed yet

    // Everything looks OK, let's attempt to read the sensors
    _i2c_model->ClearI2cDeviceEvent();

    // Is the driver on the mux?
    uint32_t mux_channel = drv->GetMuxChannel();
    if (mux_channel != 0xFFFF) {
      // Enable the bus on the mux channel
      if (drv->HasAltI2CBus()) {
        WS_DEBUG_PRINT("[i2c] Alt. Bus, MUX CH#: ");
        WS_DEBUG_PRINTLN(mux_channel)
        _i2c_bus_alt->ClearMuxChannel(); // sanity-check
        _i2c_bus_alt->SelectMuxChannel(mux_channel);
      } else {
        WS_DEBUG_PRINT("[i2c] Reg. Bus, MUX CH#: ");
        WS_DEBUG_PRINTLN(mux_channel)
        _i2c_bus_default->ClearMuxChannel(); // sanity-check
        _i2c_bus_default->SelectMuxChannel(mux_channel);
      }
    }

    for (size_t i = 0; i < sensor_count; i++) {
      // read and fill the event(s)
      // fill via SetI2cDeviceEventDeviceDescripton
      // ^ can probably do from a map->event handler

      // log event
      // send even stream (either publish or to ws_sd)
    }
  }
}