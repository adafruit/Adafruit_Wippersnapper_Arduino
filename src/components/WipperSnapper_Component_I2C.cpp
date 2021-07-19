/*!
 * @file Wippersnapper_Component_I2C.cpp
 *
 * This component initiates I2C operations
 * using the Arduino generic TwoWire driver.
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

#include "WipperSnapper_Component_I2C.h"

WipperSnapper_Component_I2C::WipperSnapper_Component_I2C(int32_t sdaPin, int32_t sclPin, int32_t portNum, uint32_t frequency) {
    // initialize using portNum
    _i2c = new TwoWire(int32_t portNum);

    // validate if pins are pulled HI
    if (digitalRead(sdaPin) == LOW) {
        pinMode(sdaPin, INPUT_PULLUP);
    }
    if (digitalRead(sclPin) == LOW) {
        pinMode(sclPin, INPUT_PULLUP);
    }
    // begin i2c
    _i2c.begin(sdaPin, sclPin);
    _i2c.setClock(frequency);
    _isInit = true;
    yield();
}

WipperSnapper_Component_I2C::~WipperSnapper_Component_I2C() {
    // todo - DTOR!
}

bool WipperSnapper_Component_I2C::scanForAddress(uint32_t address) {
    bool is_success = false;
    // TODO: scanner code
    return is_success;
}

