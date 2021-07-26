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
    // TODO: Use address propery from I2CDriver generic
    // TODO: Get bus from I2CComponent 
    WS_DEBUG_PRINT("ON PORT: ")
    WS_DEBUG_PRINTLN(this->i2cComponent->_portNum);
    //_ahtx0 = new Adafruit_AHTX0(this->i2cComponent->_i2c);
/*     if (!_ahtx0->begin()) {
        return false;
    } */
    return true;
}