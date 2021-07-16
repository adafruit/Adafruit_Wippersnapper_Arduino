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

// forward decl.
class Wippersnapper;

class WipperSnapper_I2C() {
    public:
        WipperSnapper_I2C(uint32_t frequency);
        ~WipperSnapper_I2C();
        bool scanForAddress(uint32_t address);
        bool setFrequency(uint32_t frequency);

    private:
        //TwoWire *_i2c = NULL; /*!< Default Arduino I2C bus */

};
extern Wippersnapper WS;

#endif //WIPPERSNAPPER_I2C_H