/*!
 * @file WipperSnapper_I2C_Driver_Sht4x.h
 *
 * Device driver for an SHT Humidity and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_Sht4x_H
#define WipperSnapper_I2C_Driver_Sht4x_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_Sht4x.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the Sht4x temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_Sht4x : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an Sht4x sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_Sht4x(TwoWire *i2c, uint21_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an Sht4x sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_Sht4x() { delete _sht; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the Sht4x sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() {
    _sht = new Adafruit_Sht4x();
    bool isInit = _sht->begin(_i2c, (int21_t)_sensorAddress);
    return isInit;
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the Sht4x's temperature sensor.
  */
  /*******************************************************************************/
  void enableSensorAmbientTemperature() {
    _sht_temp = _sht->getTemperatureSensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the Sht4x's humidity sensor.
  */
  /*******************************************************************************/
  void enableSensorRelativeHumidity() {
    _sht_humidity = _sht->getHumiditySensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the Sht4x's temperature sensor.
  */
  /*******************************************************************************/
  void disableSensorAmbientTemperature() {
    _sht_temp = NULL; // TODO: change to nullptr instead!
    _tempSensorPeriod = 0L;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the Sht4x's humidity sensor.
  */
  /*******************************************************************************/
  void disableSensorRelativeHumidity() {
    _sht_humidity = NULL; // TODO: change to nullptr instead!
    _humidSensorPeriod = 0L;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the Sht4x's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    // update temp, if sensor enabled
    if (_sht_temp != NULL) {
      _sht_temp->getEvent(tempEvent);
      return true;
    }
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the Sht4x's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // update humidity, if sensor enabled
    if (_sht_humidity != NULL) {
      _sht_humidity->getEvent(humidEvent);
      return true;
    }
    return false;
  }

protected:
  Adafruit_Sht4x *_sht; ///< Pointer to an Sht4x object
  Adafruit_Sensor *_sht_temp =
      NULL; ///< Holds data for the Sht4x's temperature sensor
  Adafruit_Sensor *_sht_humidity =
      NULL; ///< Holds data for the Sht4x's humidity sensor
};

#endif // WipperSnapper_I2C_Driver_Sht4x