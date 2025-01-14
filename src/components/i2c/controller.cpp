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
    std::function<drvBase *(TwoWire *, uint16_t)>;

// Map of sensor names to lambda functions that create an I2C device driver
// NOTE: This list is NOT comprehensive, it's a  subset for now
// to assess the feasibility of this approach.
static std::map<std::string, FnCreateI2CDriver> I2cFactory = {
    // Many sensors share the same driver class AHTX0
    {"aht20",
     [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_AHTX0(i2c, addr);
     }},
    {"am2301b",
     [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_AHTX0(i2c, addr);
     }},
    {"am2315c",
     [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_AHTX0(i2c, addr);
     }},
    {"dht20",
     [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_AHTX0(i2c, addr);
     }},
    {"bh1750",
     [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_BH1750(i2c, addr);
     }},
    {"bme280",
     [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_BME280(i2c, addr);
     }},
    {"bmp280", [](TwoWire *i2c, uint16_t addr) {
       return new WipperSnapper_I2C_Driver_BMP280(i2c, addr);
     }}};

drvBase *createI2CDriverByName(const char *sensorName, TwoWire *i2c, uint16_t addr, wippersnapper_i2c_I2cDeviceStatus &status) {
  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
  auto it = I2cFactory.find(sensorName);
  if (it == I2cFactory.end()) {
    status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
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
bool I2cController::IsBusOK() {
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

  if (!IsBusOK()) {
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

  // TODO: Differentiate between Add and Replace message types

  // only using the default bus, for now
  drvBase *drv = createI2CDriverByName(
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name,
      _i2c_hardware->GetI2cBus(),
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()
          ->i2c_device_description.i2c_device_mux_address,
      device_status);

  if (drv == nullptr)
    return false;

  // Configure the driver
  // TODO: We may need to refactor this method because API V2 functions
  // differently!
  // drv->configureDriver(_i2c_model->GetI2cDeviceAddOrReplaceMsg());
  _i2c_drivers.push_back(drv);
  device_status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;

  // _i2c_model->GetI2cDeviceAddOrReplaceMsg()
}