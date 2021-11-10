/*!
 * @file WipperSnapper_I2C_Driver_SCD4X.h
 *
 * Device driver for Sensirion SCD4x sensors.
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

#ifndef WipperSnapper_I2C_Driver_SCD4X_H
#define WipperSnapper_I2C_Driver_SCD4X_H

#include "WipperSnapper_I2C_Driver.h"
#include <SensirionI2CScd4x.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SCD30 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SCD4X : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SCD4x sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SCD4X(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    uint16_t error, serial0, serial1, serial2;
    setDriverType(SCD4X);
    _sensorAddress = sensorAddress;

    // TODO: The sensioron scd4x constructor is different from the
    // std adafruit libraries, the following may not work
    // https://github.com/Sensirion/arduino-i2c-scd4x/blob/master/src/SensirionI2CScd4x.cpp#L49
    Wire.begin();
    _scd4x.begin(Wire);

    // stop potentially previously started measurement
    error = _scd4x.stopPeriodicMeasurement();
    // Error trying to execute stopPeriodicMeasurement()
    if (error) {
      _isInitialized = false;
      return;
    }

    // attempt to get serial number
    error = _scd4x.getSerialNumber(serial0, serial1, serial2);
    // Error trying to execute getSerialNumber()
    if (error) {
      _isInitialized = false;
      return;
    }

    // attempt to start measurement
    error = _scd4x.startPeriodicMeasurement();
    if (error) {
      _isInitialized = false;
      return;
    }

    _isInitialized = true;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SCD4x sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_SCD4X() {
    _tempSensorPeriod = -1L;
    _humidSensorPeriod = -1L;
    _CO2SensorPeriod = -1L;
    setDriverType(UNSPECIFIED);
  }

protected:
  SensirionI2CScd4x _scd4x; ///< SCD4X object
};

#endif // WipperSnapper_I2C_Driver_SCD4X