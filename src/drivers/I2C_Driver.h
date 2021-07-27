/*!
 * @file I2C_driver.h
 *
 * Generic I2C device driver
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
#ifndef I2C_Driver_H
#define I2C_Driver_H

#include "Wippersnapper.h"
#include "components/WipperSnapper_Component_I2C.h"

class Wippersnapper;

class I2C_Driver {
    public:
        I2C_Driver(uint16_t deviceAddress);
        virtual bool initDriver();
        uint16_t address;

};
extern Wippersnapper WS;

#endif // I2C_Driver_H