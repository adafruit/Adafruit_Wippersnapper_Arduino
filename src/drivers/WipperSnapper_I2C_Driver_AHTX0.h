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
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_AHTX0(TwoWire *_i2c) : WipperSnapper_I2C_Driver() {
    isInitialized = _aht.begin(_i2c);
    _aht_temp_period = 0.0;
    _aht_humidity_period = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AHTX0 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_AHTX0() {
    _aht_temp = NULL;
    _aht_temp_period = 0.0;
    _aht_humidity = NULL;
    _aht_humidity_period = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the AHTX0's temperature sensor.
  */
  /*******************************************************************************/
  void enableTemperatureSensor() {
    _aht_temp = _aht.getTemperatureSensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the AHTX0's humidity sensor.
  */
  /*******************************************************************************/
  void enableHumiditySensor() {
    _aht_humidity = _aht.getHumiditySensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the AHTX0's temperature sensor.
  */
  /*******************************************************************************/
  void disableTemperatureSensor() {
    _aht_temp = NULL;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the AHTX0's humidity sensor.
  */
  /*******************************************************************************/
  void disableHumiditySensor() {
    _aht_humidity = NULL;
  }

  /*******************************************************************************/
  /*!
      @brief    Set the AHTX0 temperature sensor's return frequency.
      @param    tempPeriod
                The time interval at which to return new data from the AHTX's
                temperature sensor.
  */
  /*******************************************************************************/
  void setTemperatureSensorPeriod(float tempPeriod) {
    _aht_temp_period = tempPeriod;
  }

  /*******************************************************************************/
  /*!
      @brief    Set the AHTX0 humidity sensor's return frequency.
      @param    humidPeriod
                The time interval at which to return new data from the AHTX's
                humidity sensor.
  */
  /*******************************************************************************/
  void setHumiditySensorPeriod(float humidPeriod) {
    _aht_humidity_period = humidPeriod;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0 temperature sensor's return frequency.
      @returns  The time interval when new data is returned by the AHTX's
                temperature sensor.
  */
  /*******************************************************************************/
  float getTemperatureSensorPeriod() {
    return _aht_temp_period;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0 humidity sensor's return frequency.
      @returns  The time interval when new data is returned by the AHTX's
                humidity sensor.
  */
  /*******************************************************************************/
  float getHumiditySensorPeriod() {
    return _aht_humidity_period;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current temperature.
      @param    temperature
                A pointer to a temperature reading.
  */
  /*******************************************************************************/
  void updateTemperature(float *temperature) {
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
  void updateHumidity(float *humidity) {
    sensors_event_t humid;
    // update humid, if sensor enabled
    if (_aht_humidity != NULL) {
      _aht_humidity->getEvent(&humid);
      *humidity = humid.relative_humidity;
    }
  }

protected:
  Adafruit_AHTX0 _aht; ///< AHTX0 driver object
  Adafruit_Sensor *_aht_temp = NULL; ///< Holds data for the AHTX0's temperature sensor
  Adafruit_Sensor *_aht_humidity = NULL; ///< Holds data for the AHTX0's humidity sensor
  float _aht_temp_period; ///< Return frequency of the AHTX0's temperature sensor.
  float _aht_humidity_period; ///< Return frequency of the AHTX0's humidity sensor.
};

#endif // WipperSnapper_I2C_Driver_AHTX0