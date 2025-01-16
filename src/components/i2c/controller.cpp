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
using FnCreateI2CDriver = std::function<drvBase *(TwoWire *, uint16_t)>;

// Map of sensor names to lambda functions that create an I2C device driver
// NOTE: This list is NOT comprehensive, it's a  subset for now
// to assess the feasibility of this approach.
// TODO: Add in a MUX here!
static std::map<std::string, FnCreateI2CDriver> I2cFactory = {
    {"bme280", [](TwoWire *i2c, uint16_t addr) -> drvBase * {
       return new drvBme280(i2c, addr);
     }}};

drvBase *createI2CDriverByName(const char *sensorName, TwoWire *i2c,
                               uint16_t addr,
                               wippersnapper_i2c_I2cDeviceStatus &status) {
  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
  auto it = I2cFactory.find(sensorName);
  if (it == I2cFactory.end()) {
    status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }

  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;
  // Call the lambda to create the driver
  return it->second(i2c, addr);
}

/***********************************************************************/
/*!
    @brief  I2cController constructor
*/
/***********************************************************************/
I2cController::I2cController() {
  _i2c_model = new I2cModel();
  // Initialize the *default* I2C bus
  // TODO: Handle multiple buses, maybe not from here though!
  _i2c_hardware = new I2cHardware();
  _i2c_hardware->InitDefaultBus();
}

/***********************************************************************/
/*!
    @brief  I2cController destructor
*/
/***********************************************************************/
I2cController::~I2cController() {
  if (_i2c_model)
    delete _i2c_model;

  if (_i2c_hardware)
    delete _i2c_hardware;
}

/***********************************************************************/
/*!
    @brief    Returns the status of the I2C bus
    @returns  True if the I2C bus is operational, False otherwise.
*/
/***********************************************************************/
bool I2cController::IsBusStatusOK() {
  if (!_i2c_hardware->GetBusStatus() ==
      wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS)
    return false;
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
  wippersnapper_i2c_I2cDeviceStatus device_status;

  if (!IsBusStatusOK()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C bus is not operational or stuck, please "
                     "restart device!");
    return false;
  }

  // Attempt to decode an I2cDeviceAddOrReplace message
  if (!_i2c_model->DecodeI2cDeviceAddReplace(stream)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to decode I2cDeviceAddOrReplace message!");
    return false;
  }
  wippersnapper_i2c_I2cDeviceDescriptor device_descriptor =
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_description;

  // TODO: Handle Replace messages by implementing a Remove handler first...then
  // proceed to adding a new device

  // TODO: This is only using the default bus, for now
  drvBase *drv = createI2CDriverByName(
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name,
      _i2c_hardware->GetI2cBus(), device_descriptor.i2c_device_mux_address,
      device_status);
  if (drv != nullptr) {
    // Configure and add the new driver to the controller
    drv->ConfigureSensorTypes(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_sensor_types,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()
            ->i2c_device_sensor_types_count);
    _i2c_drivers.push_back(drv);
  }

  if (WsV2._sdCardV2->isModeOffline())
    return true;

  // Publish I2cDeviceAddedOrReplaced message back to IO
  if (!_i2c_model->encodeMsgI2cDeviceAddedorReplaced(
          device_descriptor, _i2c_hardware->GetBusStatus(), device_status)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to encode I2cDeviceAddedorReplaced message!");
    return false;
  }

  if (!WsV2.PublishSignal(
          wippersnapper_signal_DeviceToBroker_i2c_device_added_replaced_tag,
          _i2c_model->GetMsgI2cDeviceAddedOrReplaced())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish I2cDeviceAddedorReplaced "
                     "message to IO!");
    return false;
  }
  return true;
}