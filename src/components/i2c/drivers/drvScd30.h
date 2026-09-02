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

/*!
    @brief  Class that provides a driver interface for the SCD30 sensor.
*/
class drvScd30 : public drvBase {

public:
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
  drvScd30(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the SCD30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _scd = new Adafruit_SCD30();
    return _scd->begin((uint8_t)_address, _i2c);
  }

  /*!
      @brief    Checks if the sensor has a new measurement ready to read.
      @returns  True if a new measurement is ready, False otherwise.
  */
  bool IsSensorReady() { return _scd->dataReady(); }

  /*!
      @brief    Reads the SCD30's CO2, temperature and humidity in one
                transaction so all metrics reflect the same sample, caching
                the results. Serves the cached sample if the last read was
                under one second ago, or if no new data is ready yet.
      @returns  True if a valid sample is cached, False if no sample has been
                read yet (or the read failed).
  */
  bool ReadSensorData() override {
    if (HasBeenReadInLastSecond())
      return _have_data;

    if (IsSensorReady() && _scd->read()) {
      _CO2 = _scd->CO2;
      _humidity = _scd->relative_humidity;
      _temperature = _scd->temperature;
      _last_read = millis();
      _have_data = true;
    }
    return _have_data;
  }

  /*!
      @brief    Gets the SCD30's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // check if sensor is enabled and data is available
    if (!ReadSensorData()) {
      return false;
    }

    tempEvent->temperature = _temperature;
    return true;
  }

  /*!
      @brief    Gets the SCD30's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // check if sensor is enabled and data is available
    if (!ReadSensorData()) {
      return false;
    }

    humidEvent->relative_humidity = _humidity;
    return true;
  }

  /*!
      @brief    Gets the SCD30's current CO2 reading.
      @param    co2Event
                  Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventCO2(sensors_event_t *co2Event) {
    // check if sensor is enabled and data is available
    if (!ReadSensorData()) {
      return false;
    }

    co2Event->CO2 = _CO2;
    return true;
  }

protected:
  Adafruit_SCD30 *_scd = nullptr; ///< SCD30 driver object
  float _temperature = NAN;       ///< Temperature
  float _humidity = NAN;          ///< Relative Humidity
  float _CO2 = NAN;               ///< CO2
};

#endif // drvScd30