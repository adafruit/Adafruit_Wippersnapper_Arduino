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
#include <Adafruit_AHTX0.h>

class I2C_Driver {
    public:
        I2C_Driver(uint16_t deviceAddress, TwoWire *i2c);
        ~I2C_Driver();
        // GENERIC, virtual
        bool initSensor();
        void setPeriod(float periodMs);
        // AHT-Specific functions
        void enableSensorTemperature();
        void enableSensorHumidity();

        TwoWire *_i2c = NULL;
        Adafruit_AHTX0 *_ahtx0 = NULL;
    private:
        // Generic
        float _pollPeriod;
        // AHT-Specific
        Adafruit_Sensor *_aht_temperature = NULL;
        Adafruit_Sensor *_aht_humidity = NULL;
};

#endif // I2C_Driver_H