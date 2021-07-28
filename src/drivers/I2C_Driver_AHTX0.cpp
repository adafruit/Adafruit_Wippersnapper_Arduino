/*!
 * @file I2C_Driver_AHTX0.cpp
 *
 * Subclass for an AHT10 & AHT20 Humidity and Temperature Sensor.
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
#include "I2C_Driver_AHTX0.h"

/* // Generic
I2C_Driver_AHTX0::I2C_Driver_AHTX0(uint16_t deviceAddress, TwoWire *i2c) {
    // Base implementation
    WS_DEBUG_PRINTLN("I2CDriver Initialized!");
    WS_DEBUG_PRINT("I2CDriver Device Addr: ");WS_DEBUG_PRINTLN(deviceAddress);
    _i2c = i2c;
    _pollPeriod = 0.0;
}

// Generic
I2C_Driver_AHTX0::~I2C_Driver_AHTX0() {
    // Base implementation
}
 */

// Specific, virtual
bool I2C_Driver_AHTX0::initSensor() {
  bool is_success = true; 
  _ahtx0 = new Adafruit_AHTX0();
  if (!_ahtx0->begin(_i2c)) {
      WS_DEBUG_PRINTLN("Error: AHTx0 not initialized");
      is_success = false;
  };
  // TODO: Enable temp and humid too
  // enableSensorTemperature();
  //enableSensorHumidity();
  WS_DEBUG_PRINTLN("AHT initialized successfully!");
  return is_success;
}

// Specific, virtual
void I2C_Driver_AHTX0::pollSensor() {
  // TODO - Validate sensors and poll them
}

// AHT-Specific
void I2C_Driver_AHTX0::enableSensorTemperature() {
  _ahtTemperature = _ahtx0->getTemperatureSensor();
  WS_DEBUG_PRINTLN("Enabled AHTX0 temperature sensor");
}

void I2C_Driver_AHTX0::enableSensorHumidity() {
  _ahtHumidity = _ahtx0->getHumiditySensor();
  WS_DEBUG_PRINTLN("Enabled AHTX0 humidity sensor");
}