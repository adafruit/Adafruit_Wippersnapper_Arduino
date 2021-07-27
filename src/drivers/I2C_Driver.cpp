/*!
 * @file I2C_Driver.cpp
 *
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "I2C_Driver.h"

I2C_Driver::I2C_Driver(uint16_t deviceAddress, TwoWire *i2c) {
    // Base implementation
    WS_DEBUG_PRINTLN("I2CDriver Initialized!");
    WS_DEBUG_PRINT("I2CDriver Device Addr: ");WS_DEBUG_PRINTLN(deviceAddress);
    _i2c = i2c;
    _pollPeriod = 0.0;
}

I2C_Driver::~I2C_Driver() {
    // Base implementation
}

bool I2C_Driver::initSensor() {
  bool is_success = true; 
  _ahtx0 = new Adafruit_AHTX0();
  if (!_ahtx0->begin(_i2c)) {
      WS_DEBUG_PRINTLN("Error: AHTx0 not initialized");
      is_success = false;
  };
  WS_DEBUG_PRINTLN("AHT initialized successfully!");
  return is_success;
}

void I2C_Driver::setPeriod(float periodMs) {
  _pollPeriod = periodMs;
}

// AHT-Specific
void I2C_Driver::enableSensorTemperature() {
  _aht_temperature = _ahtx0->getTemperatureSensor();
}

void I2C_Driver::enableSensorHumidity() {
  _aht_temperature = _ahtx0->getHumiditySensor();
}