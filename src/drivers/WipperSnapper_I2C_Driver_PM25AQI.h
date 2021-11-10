/*!
 * @file WipperSnapper_I2C_Driver_PM25AQI.h
 *
 * Device driver for PMSx Air Quality Sensors
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_PM25AQI_H
#define WipperSnapper_I2C_Driver_PM25AQI_H

#include "WipperSnapper_I2C_Driver.h"
#include "Adafruit_PM25AQI.h"

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SCD30 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_PM25AQI : public WipperSnapper_I2C_Driver {

public:
  Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
  /*******************************************************************************/
  /*!
      @brief    Constructor for a PM25AQI sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_PM25AQI(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    setDriverType(PM25AQI);
    _sensorAddress = sensorAddress;

    if (! aqi.begin_I2C(_i2c)) {
        _isInitialized = false;
        return;
    }

    _isInitialized = true;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a PM25AQI sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_PM25AQI() {
    // TODO
    setDriverType(UNSPECIFIED);
  }

protected:
  PM25_AQI_Data _aqi_data;
};

#endif // WipperSnapper_I2C_Driver_PM25AQI