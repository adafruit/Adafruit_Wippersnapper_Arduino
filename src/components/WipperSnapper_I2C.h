/*!
 * @file WipperSnapper_I2C.h
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
#ifndef WIPPERSNAPPER_I2C_H
#define WIPPERSNAPPER_I2C_H

#include "Wippersnapper.h"
#include <Wire.h>

// forward decl.
class Wippersnapper;

class WipperSnapper_I2C() {
    public:
        WipperSnapper_I2C(uint32_t frequency, int32_t sdaPin, int32_t sclPin, int32_t busId);
        ~WipperSnapper_I2C();
        bool scanForAddress(uint32_t address);
        bool setFrequency(uint32_t frequency);

    private:
        TwoWire *_i2cOne;
        TwoWire *_i2cTwo;
        bool isInitI2COne; // True if I2C w/busId 1 already initialized
        bool isInitI2CTwo; // True if I2C w/busId 2 already initialized

};
extern Wippersnapper WS;

#endif //WIPPERSNAPPER_I2C_H