/*!
 * @file WipperSnapper_I2C_Driver_PM25AQI.h
 *
 * Device driver for the PM2.5 Air Quality Sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_PM25AQI_H
#define WipperSnapper_I2C_Driver_PM25AQI_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_PM25AQI.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the PM25 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_PM25AQI : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a PM25 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_PM25AQI(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the PM25 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _pm25 = new Adafruit_PM25AQI(_i2c);
    // Wait one second for sensor to boot up!
    delay(1000);
    // connect over i2c
    if (!_pm25->begin_I2C())
      return false;

    return true;
  }

protected:
  Adafruit_PM25AQI *_pm25; ///< PM2.5 sensor driver
};

#endif // WipperSnapper_I2C_Driver_PM25AQI