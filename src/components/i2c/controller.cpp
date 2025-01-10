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


// WipperSnapper_I2C_Driver(TwoWire *i2c, uint16_t sensorAddress) {
std::map <std::string, WipperSnapper_I2C_Driver> i2c_drivers = {
    {"aht20", &WipperSnapper_I2C_Driver_AHTX0},
    {"am2301b", &WipperSnapper_I2C_Driver_AHTX0},
    {"am2315c", &WipperSnapper_I2C_Driver_AHTX0},
    {"dht20", &WipperSnapper_I2C_Driver_AHTX0},
    {"bh1750", &WipperSnapper_I2C_Driver_BH1750},
    {"bme280", &WipperSnapper_I2C_Driver_BME280},
    {"bmp280", &WipperSnapper_I2C_Driver_BMP280}
};

/***********************************************************************/
/*!
    @brief  I2cController constructor
*/
/***********************************************************************/
I2cController::I2cController() {
  _i2c_model = new I2cModel();
  // Initialize the *default* I2C bus
  _i2c_hardware = new I2cHardware();
  _i2c_hardware->InitDefaultBus();
  // NOTE: In the handle() functions, we'll need to
  // check the value of GetBusStatus() elsewhere in the handlers
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

bool I2cController::Handle_I2cDeviceAddOrReplace(pb_istream_t *stream) {
    if (! _i2c_model->DecodeI2cDeviceAddReplace()) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2cDeviceAddOrReplace message!");
        return false;
    }

    // _i2c_model->GetI2cDeviceAddOrReplaceMsg()
}