/*!
 * @file Wippersnapper_I2C.cpp
 *
 * This file provides functions for interfacing
 * with the I2C bus.
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

#include "WipperSnapper_I2C.h"

WipperSnapper_I2C::WipperSnapper_I2C(uint32_t frequency, int32_t sdaPin, int32_t sclPin, int32_t busId) {
    // check if sda/scl have pullups enabled
    if (digitalRead(sdaPin) == LOW) {
        pinMode(sdaPin, INPUT_PULLUP);
    }
    if (digitalRead(sclPin) == LOW) {
        pinMode(sclPin, INPUT_PULLUP);
    }

    // Initialize default I2C bus (busId = 1)
    if (busId == 1 && isInitI2COne = false) {
        _i2cOne->begin(sdaPin, sclPin, frequency);
    } // Initialize second I2C bus (busId = 2)
    else if (busId == 1 && isInitI2CTwo = false) {
        _i2cTwo->begin(sdaPin, sclPin, frequency);
    } else {
        WS_DEBUG_PRINTLN("Skipping initialization, I2C bus already configured.");
    }

}

~WipperSnapper_I2C::WipperSnapper_I2C() {
    // todo - DTOR!
}

bool WipperSnapper_I2C::scanForAddress(uint32_t address) {
    // TODO: scanner code
}

bool WipperSnapper_I2C::setFrequency(uint32_t frequency) {
    // TODO: frequency setter code
}
