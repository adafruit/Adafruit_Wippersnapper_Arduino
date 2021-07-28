/*!
 * @file I2C_Driver.cpp
 *
 * Base class for a generic I2C sensor device driver.
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

// Generic
I2C_Driver::I2C_Driver(uint16_t deviceAddress, TwoWire *i2c) {
    // Base implementation
    WS_DEBUG_PRINTLN("I2CDriver Initialized!");
    WS_DEBUG_PRINT("I2CDriver Device Addr: ");WS_DEBUG_PRINTLN(deviceAddress);
    _i2c = i2c;
    _pollPeriod = 0.0;
}

// Generic
I2C_Driver::~I2C_Driver() {
    // Base implementation
}

// Generic
void I2C_Driver::setPeriod(float periodMs) {
  _pollPeriod = periodMs;
}

bool I2C_Driver::initSensor() {
    // base calls/*  */
    return true;
}

// Specific, virtual
void I2C_Driver::pollSensor() {
  // TODO - Validate sensors and poll them
}