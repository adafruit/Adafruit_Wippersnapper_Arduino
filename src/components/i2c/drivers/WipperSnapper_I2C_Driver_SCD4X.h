/*!
 * @file WipperSnapper_I2C_Driver_SCD4x.h
 *
 * Device driver for the SCD4X CO2, Temperature, and Humidity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Marni Brewster 2022
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_SCD4X_H
#define WipperSnapper_I2C_Driver_SCD4X_H

#include "WipperSnapper_I2C_Driver.h"
#include <SensirionI2CScd4x.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SCD40 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SCD4X : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SCD40 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SCD4X(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SCD40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _scd = new SensirionI2CScd4x();
    _scd->begin(*_i2c);

    // stop previously started measurement
    if (_scd->stopPeriodicMeasurement() != 0) {
      return false;
    }

    // start measurements
    if (_scd->startPeriodicMeasurement() != 0) {
      return false;
    }

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  bool alreadyRecentlyRead() {
    return _lastRead != 0 && millis() - _lastRead < 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if the sensor is ready to be read
      @returns  True if the sensor is ready, False otherwise.
  */
  /*******************************************************************************/
  bool sensorReady() {
    bool isDataReady = false;
    uint16_t error = _scd->getDataReadyFlag(isDataReady);
    if (error != 0 || !isDataReady) {
      // failed, one more quick attempt
      delay(100);
      error = _scd->getDataReadyFlag(isDataReady);
      if (error != 0 || !isDataReady) {
        return false;
      }
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool readSensorData() {
    // dont read sensor more than once per second
    if (alreadyRecentlyRead()) {
      return true;
    }

    if (!sensorReady()) {
      return false;
    }


    // Read SCD4x measurement
    uint16_t error = _scd->readMeasurement(_co2, _temperature, _humidity);
    if (error != 0 || _co2 == 0) {
      return false;
    }
    _lastRead = millis();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD40's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // read all sensor measurements
    if (!readSensorData()) {
      return false;
    }

    tempEvent->temperature = _temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD40's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // read all sensor measurements
    if (!readSensorData()) {
      return false;
    }

    humidEvent->relative_humidity = _humidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD40's current CO2 reading.
      @param    co2Event
                  Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    // read all sensor measurements
    if (!readSensorData()) {
      return false;
    }

    co2Event->CO2 = (float)_co2;
    return true;
  }

protected:
  SensirionI2CScd4x *_scd = nullptr; ///< SCD4x driver object
  uint16_t _co2 = 0;                 ///< SCD4x co2 reading
  float _temperature = 20.0f;           ///< SCD4x temperature reading
  float _humidity = 50.0f;              ///< SCD4x humidity reading
  ulong _lastRead = 0;               ///< Last time the sensor was read
};

#endif // WipperSnapper_I2C_Driver_SCD4X