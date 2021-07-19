/*!
 * @file WipperSnapper_Component_I2C.h
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
#ifndef WipperSnapper_Component_I2C_H
#define WipperSnapper_Component_I2C_H

#include "Wippersnapper.h"
#include <Wire.h>

// forward decl.
class Wippersnapper;

class WipperSnapper_Component_I2C {
    public:
        WipperSnapper_Component_I2C(int32_t sdaPin, int32_t sclPin, int32_t portNum=0, uint32_t frequency=100000);
        ~WipperSnapper_Component_I2C();
        bool scanForAddress(uint32_t address);

    private:
        TwoWire *_i2c = NULL;
        bool _isInit;
};
extern Wippersnapper WS;

#endif //WipperSnapper_Component_I2C_H