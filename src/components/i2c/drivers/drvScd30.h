/*!
 * @file drvScd30.h
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

#ifndef DRV_SCD30_H
#define DRV_SCD30_H

#include "drvBase.h"
#include <Adafruit_SCD30.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SCD30 sensor.
*/
/**************************************************************************/
class drvScd30 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SCD30 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvScd30(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel, const char* driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SCD30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _scd = new Adafruit_SCD30();
    return _scd->begin((uint8_t)_address, _i2c);
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
    if (!_scd->dataReady())
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
    if (!_scd->dataReady())
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
    if (!_scd->dataReady())
      return false;

    co2Event->CO2 = _scd->CO2;
    return true;
  }

protected:
  Adafruit_SCD30 *_scd; ///< SCD30 driver object
};

#endif // drvScd30