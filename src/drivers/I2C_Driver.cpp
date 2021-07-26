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

I2C_Driver::I2C_Driver(WipperSnapper_Component_I2C *i2cComponent, uint16_t i2cAddress) {
    _i2cComponent = i2cComponent;
    _i2cAddress = i2cAddress;
}

bool I2C_Driver::init() {
    // Base implementation of I2C device driver init func.
    return true;
}