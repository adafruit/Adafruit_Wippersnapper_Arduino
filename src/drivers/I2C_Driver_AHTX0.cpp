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
#include "I2C_Driver_AHTX0.h"


bool I2C_Driver_AHTX0::initDriver() {
    WS_DEBUG_PRINTLN("Initialize AHTX0 sensor");
    //_ahtx0 = new Adafruit_AHTX0(this->_i2cAddress);
    return true;
}