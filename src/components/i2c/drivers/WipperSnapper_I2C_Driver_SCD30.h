/*!
 * @file WipperSnapper_I2C_Driver_SCD30.h
 *
 * Device driver for the SCD30 CO2, Temperature, and Humidity sensor.
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

#ifndef WipperSnapper_I2C_Driver_SCD30_H
#define WipperSnapper_I2C_Driver_SCD30_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SCD30.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SCD30 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SCD30 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SCD30 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SCD30(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SCD30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _scd = new Adafruit_SCD30();
    return _scd->begin((uint8_t)_sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  bool HasBeenReadInLastSecond() {
    return _lastRead != 0 && millis() - _lastRead < 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if the sensor is ready to be read
      @returns  True if the sensor is ready, False otherwise.
  */
  /*******************************************************************************/
  bool IsSensorReady() {
    if (!_scd->dataReady()) {
      // failed, one more quick attempt
      delay(100);
      if (!_scd->dataReady()) {
        return false;
      }
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the SCD30 sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    // dont read sensor more than once per second
    if (HasBeenReadInLastSecond()) {
      return true;
    }

    if (!IsSensorReady()) {
      return false;
    }

    if (!_scd->getEvent(&_humidity, &_temperature)) {
      return false;
    }
    _CO2.CO2 = _scd->CO2;
    _lastRead = millis();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD30's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // check if sensor is enabled and data is available
    if (!ReadSensorData()) {
      return false;
    }

    *tempEvent = _temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD30's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // check if sensor is enabled and data is available
    if (!ReadSensorData()) {
      return false;
    }

    *humidEvent = _humidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD30's current CO2 reading.
      @param    co2Event
                  Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    // check if sensor is enabled and data is available
    if (!ReadSensorData()) {
      return false;
    }

    *co2Event = _CO2;
    return true;
  }

protected:
  Adafruit_SCD30 *_scd = nullptr;     ///< SCD30 driver object
  ulong _lastRead = 0uL;              ///< Last time the sensor was read
  sensors_event_t _temperature = {0}; ///< Temperature
  sensors_event_t _humidity = {0};    ///< Relative Humidity
  sensors_event_t _CO2 = {0};         ///< CO2
};

#endif // WipperSnapper_I2C_Driver_SCD30