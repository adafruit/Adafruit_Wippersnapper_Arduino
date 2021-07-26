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
#ifndef I2C_Driver_AHTX0_H
#define I2C_Driver_AHTX0_H
#include "Wippersnapper.h"
#include "I2C_Driver.h"
#include <Adafruit_AHTX0.h>

// forward decl.
class Wippersnapper;

class I2C_Driver_AHTX0 : public I2C_Driver {
    public:
        // Generic overrides
        bool initDriver() override;
        // Sensor-specific, TODO
    private:
        Adafruit_AHTX0 *_ahtx0 = NULL;
        Adafruit_Sensor *_ahtx0_humidity = NULL;
        Adafruit_Sensor *_ahtx0_temperature = NULL;
};
extern Wippersnapper WS;

#endif // I2C_Driver_AHTX0_H