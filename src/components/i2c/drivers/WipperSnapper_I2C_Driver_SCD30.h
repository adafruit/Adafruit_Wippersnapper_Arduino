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
      @brief    Gets the SCD30's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // check if sensor is enabled and data is available
    if (_tempSensorPeriod != 0 && (!_scd->dataReady()))
      return false;

    // attempt to get temperature data
    sensors_event_t humidEvent;
    if (!_scd->getEvent(&humidEvent, tempEvent))
      return false;

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
    if (_humidSensorPeriod != 0 && (!_scd->dataReady()))
      return false;

    // attempt to get temperature data
    sensors_event_t tempEvent;
    if (!_scd->getEvent(humidEvent, &tempEvent))
      return false;

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
    if (_CO2SensorPeriod != 0 && (!_scd->dataReady()))
      return false;
    // TODO: This is a TEMPORARY HACK, we need to add CO2 type to
    // adafruit_sensor
    co2Event->data[0] = _scd->CO2;
    return true;
  }

protected:
  Adafruit_SCD30 *_scd; ///< SCD30 driver object
};

#endif // WipperSnapper_I2C_Driver_SCD30