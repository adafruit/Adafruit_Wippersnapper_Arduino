/*!
 * @file WipperSnapper_I2C_Driver_BME280.h
 *
 * Device driver for an AHT Humidity and Temperature sensor.
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

#ifndef WipperSnapper_I2C_Driver_BME280_H
#define WipperSnapper_I2C_Driver_BME280_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BME280.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BME280 temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BME280 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BME280 sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the BME280 sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BME280(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    setDriverType(BME280); // sets the type of I2C_Driver
    _isInitialized = _bme.begin(sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BME280 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_BME280() {
    _bme_temp = NULL;
    _bme_humidity = NULL;
    _bme_pressure = NULL;
    _tempSensorPeriod = 0.0L;
    _humidSensorPeriod = 0.0L;
    _pressureSensorPeriod = 0.0L;
    setDriverType(UNSPECIFIED);
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the BME280's temperature sensor.
  */
  /*******************************************************************************/
  void enableTemperatureSensor() { _bme_temp = _bme.getTemperatureSensor(); }

  /*******************************************************************************/
  /*!
      @brief    Enables the BME280's humidity sensor.
  */
  /*******************************************************************************/
  void enableHumiditySensor() { _bme_humidity = _bme.getHumiditySensor(); }

  /*******************************************************************************/
  /*!
      @brief    Enables the BME280's humidity sensor.
  */
  /*******************************************************************************/
  void enablePressureSensor() { _bme_pressure = _bme.getPressureSensor(); }

  /*******************************************************************************/
  /*!
      @brief    Disables the BME280's temperature sensor.
  */
  /*******************************************************************************/
  void disableTemperatureSensor() {
    _bme_temp = NULL;
    _tempSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the BME280's humidity sensor.
  */
  /*******************************************************************************/
  void disableHumiditySensor() {
    _bme_humidity = NULL;
    _humidSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the BME280's pressure sensor.
  */
  /*******************************************************************************/
  void disablePressureSensor() {
    _bme_pressure = NULL;
    _pressureSensorPeriod = 0.0;
  }

protected:
  Adafruit_BME280 _bme;
  Adafruit_Sensor *_bme_temp = NULL;
  Adafruit_Sensor *_bme_pressure = NULL;
  Adafruit_Sensor *_bme_humidity = NULL;
};

#endif // WipperSnapper_I2C_Driver_BME280