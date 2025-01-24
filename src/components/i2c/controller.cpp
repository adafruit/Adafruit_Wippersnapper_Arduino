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
// TODO: Add in a MUX here!
static std::map<std::string, FnCreateI2CDriver> I2cFactory = {
    {"bme280",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBme280(i2c, addr, mux_channel, driver_name);
     }},
    {"pca9546",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvPca9546(i2c, addr, mux_channel, driver_name);
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
    @brief    Returns the MUX driver for a given address
    @param    mux_address
                The I2C address of the MUX
    @returns  The MUX driver, if found, nullptr otherwise.
*/
/***********************************************************************/
drvBase *I2cController::GetMuxDrv(uint32_t mux_address) {
  for (auto *driver : _i2c_drivers) {
    // TODO: Refactor the first part, strcmp
    // When I implement >1 mux type, how to do this might make more sense..
    if (strcmp(driver->GetDrvName(), "pca9546") == 0 &&
        driver->GetAddress() == mux_address) {
      WS_DEBUG_PRINTLN("Found MUX driver!")
      return driver;
    }
  }
  WS_DEBUG_PRINTLN("No MUX driver found!")
  return nullptr;
}

/***********************************************************************/
/*!
    @brief    Configures the MUX channel
    @param    mux_address
                The I2C address of the MUX
    @param    mux_channel
                The MUX channel to select
    @returns  True if the MUX channel was configured successfully, False
              otherwise.
*/
/***********************************************************************/
bool I2cController::ConfigureMuxChannel(uint32_t mux_address,
                                        uint32_t mux_channel) {
  drvBase *muxDriver = GetMuxDrv(mux_address);
  if (muxDriver == nullptr)
    return false;
  muxDriver->SelectMUXChannel(mux_channel);
  return true;
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

  // Does the device's descriptor have a mux channel?
  // TODO: We will need to modify the sdconfig parser to reflect "no value" set
  // == the 0xFFFF magic value
  if (device_descriptor.i2c_mux_channel != 0xFFFF) {
    if (!ConfigureMuxChannel(device_descriptor.i2c_mux_address,
                             device_descriptor.i2c_mux_channel)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Did not find MUX - Was it created first?");
      device_status =
          wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_NOT_FOUND;
      if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
        return false;
      return true;
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
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name, bus,
      device_descriptor.i2c_device_address, device_descriptor.i2c_mux_channel,
      device_status);

  // TODO: Clean up for clarity - confusing checks and returns
  if (drv != nullptr) {
    WS_DEBUG_PRINTLN("OK! Configuring sensor types...");
    drv->ConfigureSensorTypes(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_sensor_types,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()
            ->i2c_device_sensor_types_count);
    if (drv->begin()) {
      _i2c_drivers.push_back(drv);
      WS_DEBUG_PRINTLN("[i2c] I2C driver added to controller: ");
      WS_DEBUG_PRINTLN(
          _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name);
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