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

WipperSnapper_Component_I2C::WipperSnapper_Component_I2C() {
    _i2c= &Wire;
    // TODO: allow 2nd bus to be used for ESP32
}

~WipperSnapper_Component_I2C::WipperSnapper_Component_I2C() {
    // todo - DTOR!
}

bool WipperSnapper_Component_I2C::initI2C(uint32_t frequency, int32_t sdaPin, int32_t sclPin, int32_t busId) {
    // validate if pins are pulled HI
    if (digitalRead(sdaPin) == LOW) {
        pinMode(sdaPin, INPUT_PULLUP);
    }
    if (digitalRead(sclPin) == LOW) {
        pinMode(sclPin, INPUT_PULLUP);
    }
    // init bus
    _i2c.begin(sdaPin, sclPin);
    _i2c.setClock(frequency);
    _i2cInitialized = true;
    yield();
}

bool WipperSnapper_Component_I2C::scanForAddress(uint32_t address) {
    // TODO: scanner code
}

bool WipperSnapper_Component_I2C::setFrequency(uint32_t frequency, int32 busId) {
    // TODO!
}
