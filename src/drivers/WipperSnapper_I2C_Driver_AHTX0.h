/*!
 * @file WipperSnapper_I2C_Driver_AHTX0.h
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

#ifndef WipperSnapper_I2C_Driver_AHTX0_H
#define WipperSnapper_I2C_Driver_AHTX0_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_AHTX0.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the AHTX0 temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_AHTX0 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an AHTX0 sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the AHTX0 sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_AHTX0(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    setDriverType(AHTX0); // sets the type of I2C_Driver
    _sensorAddress = sensorAddress;
    _isInitialized = _aht.begin(_i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AHTX0 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_AHTX0() {
    _aht_temp = NULL;
    _tempSensorPeriod = -1L;
    _aht_humidity = NULL;
    _humidSensorPeriod = -1L;
    setDriverType(UNSPECIFIED);
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the AHTX0's temperature sensor.
  */
  /*******************************************************************************/
  void enableTemperatureSensor() { _aht_temp = _aht.getTemperatureSensor(); }

  /*******************************************************************************/
  /*!
      @brief    Enables the AHTX0's humidity sensor.
  */
  /*******************************************************************************/
  void enableHumiditySensor() { _aht_humidity = _aht.getHumiditySensor(); }

  /*******************************************************************************/
  /*!
      @brief    Disables the AHTX0's temperature sensor.
  */
  /*******************************************************************************/
  void disableTemperatureSensor() {
    _aht_temp = NULL;
    _tempSensorPeriod = -1L;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the AHTX0's humidity sensor.
  */
  /*******************************************************************************/
  void disableHumiditySensor() {
    _aht_humidity = NULL;
    _humidSensorPeriod = -1L;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current temperature.
      @param    temperature
                A pointer to a temperature reading.
  */
  /*******************************************************************************/
  void updateTempSensor(float *temperature) {
    sensors_event_t temp;
    // update temp, if sensor enabled
    if (_aht_temp != NULL) {
      _aht_temp->getEvent(&temp);
      *temperature = temp.temperature;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current humidity.
      @param    humidity
                A pointer to a humidity reading.
  */
  /*******************************************************************************/
  void updateHumidSensor(float *humidity) {
    sensors_event_t humid;
    // update humid, if sensor enabled
    if (_aht_humidity != NULL) {
      _aht_humidity->getEvent(&humid);
      *humidity = humid.relative_humidity;
    }
  }

protected:
  Adafruit_AHTX0 _aht; ///< AHTX0 driver object
  Adafruit_Sensor *_aht_temp =
      NULL; ///< Holds data for the AHTX0's temperature sensor
  Adafruit_Sensor *_aht_humidity =
      NULL; ///< Holds data for the AHTX0's humidity sensor
};

#endif // WipperSnapper_I2C_Driver_AHTX0