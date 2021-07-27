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

class I2C_Driver {
    public:
        I2C_Driver(uint16_t deviceAddress, TwoWire *i2c);
        ~I2C_Driver();

        TwoWire *i2c = NULL;
};

#endif // I2C_Driver_H