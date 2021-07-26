/*!
 * @file I2C_driver.h
 *
 * I2C device driver for AHTX0 temperature and humidity sensors.
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
#include "I2C_Driver.h"
#include <Adafruit_AHTX0.h>

// forward decl.
class Wippersnapper;

class I2C_Driver_AHTX0 : public I2C_Driver {
    public:
        //I2C_Driver_AHTX0(WipperSnapper_Component_I2C *i2cComponent, uint16_t i2cAddress);
        bool init() override;
    private:
        Adafruit_AHTX0 *_ahtx0 = NULL;
};
extern Wippersnapper WS;

#endif // I2C_Driver_H