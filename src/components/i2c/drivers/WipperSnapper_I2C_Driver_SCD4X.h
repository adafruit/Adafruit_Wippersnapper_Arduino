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
    if (_scd->stopPeriodicMeasurement())
      return false;

    // start measurements
    if (_scd->startPeriodicMeasurement())
      return false;

    return true;
  }

  /********************************************************************************/
  /*!
      @brief    Attempts to read the SCD4x's sensor measurements
      @returns  True if the measurements were read without errors, False
                if read errors occured or if sensor did not have data ready.
  */
  /********************************************************************************/
  bool readSensorMeasurements() {
    uint16_t error;
    bool isDataReady = false;
    delay(100);

    // Check if data is ready
    error = _scd->getDataReadyStatus(isDataReady);
    if (error || !isDataReady)
      return false;

    // Read SCD4x measurement
    error = _scd->readMeasurement(_co2, _temperature, _humidity);
    if (error || _co2 == 0)
      return false;

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
    if (!readSensorMeasurements())
      return false;

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
    if (!readSensorMeasurements())
      return false;

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
    if (!readSensorMeasurements())
      return false;

    co2Event->CO2 = (float)_co2;
    return true;
  }

protected:
  SensirionI2cScd4x *_scd; ///< SCD4x driver object
  uint16_t _co2;           ///< SCD4x co2 reading
  float _temperature;      ///< SCD4x temperature reading
  float _humidity;         ///< SCD4x humidity reading
};

#endif // WipperSnapper_I2C_Driver_SCD4X
