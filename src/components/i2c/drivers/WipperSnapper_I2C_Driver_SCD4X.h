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
#include <SensirionI2cScd4x.h>
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
    _lastRead = 0;
    _temperature = 20.0;
    _humidity = 50.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SCD40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _scd = new SensirionI2cScd4x();
    _scd->begin(*_i2c, _sensorAddress);

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
  bool hasBeenReadInLastSecond() {
    return _lastRead != 0 && millis() - _lastRead < 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if the sensor is ready to be read
      @returns  True if the sensor is ready, False otherwise.
  */
  /*******************************************************************************/
  bool isSensorReady() {
    bool isDataReady = false;
    uint16_t error = _scd->getDataReadyStatus(isDataReady);
    if (error != 0 || !isDataReady) {
      // failed, one more quick attempt
      delay(100);
      error = _scd->getDataReadyStatus(isDataReady);
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
    if (hasBeenReadInLastSecond()) {
      return true;
    }

    if (!isSensorReady()) {
      return false;
    }

    // Read SCD4x measurement
    uint16_t co2 = 0;
    float temperature = 0;
    float humidity = 0;
    int16_t error = _scd->readMeasurement(co2, temperature, humidity);
    if (error != 0 || co2 == 0) {
      return false;
    }
    _CO2.CO2 = co2;
    _temperature.temperature = temperature;
    _humidity.relative_humidity = humidity;
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

    tempEvent = &_temperature;
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

    humidEvent = &_humidity;
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

    co2Event = &_CO2;
    return true;
  }

protected:
  SensirionI2cScd4x *_scd = nullptr;  ///< SCD4x driver object
  sensors_event_t _temperature = {0}; ///< Temperature
  sensors_event_t _humidity = {0};    ///< Relative Humidity
  sensors_event_t _CO2 = {0};         ///< CO2
  ulong _lastRead = 0;                ///< Last time the sensor was read
};

#endif // WipperSnapper_I2C_Driver_SCD4X_H