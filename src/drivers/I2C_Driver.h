/*!
 * @file I2C_driver.h
 *
 * Base class for a generic I2C sensor device driver.
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
#include <Adafruit_AHTX0.h>

class I2C_Driver {
    public:
        I2C_Driver(uint16_t deviceAddress, TwoWire *i2c);
        ~I2C_Driver();
        // GENERIC, shared
        void setPeriod(float periodMs);
        // AHT-Specific functions, virtual
        bool initSensor();
        void pollSensor();
        void enableSensorTemperature();
        void enableSensorHumidity();

        // Generic
        TwoWire *_i2c = NULL;
        // Specific (can be private maybe?)
        Adafruit_AHTX0 *_ahtx0 = NULL;
    private:
        // Generic
        float _pollPeriod;
        // AHT-Specific sensor properties
        Adafruit_Sensor *_ahtTemperature = NULL;
        Adafruit_Sensor *_ahtHumidity = NULL;
};

#endif // I2C_Driver_H