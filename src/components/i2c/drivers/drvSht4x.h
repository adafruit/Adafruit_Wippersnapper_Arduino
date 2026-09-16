/*!
 * @file drvSht4x.h
 *
 * Device driver for the SHT4X Temperature and Humidity Sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Marni Brewster 2022 for Adafruit Industries.
 * Copyright (c) Tyeth Gundry 2022. Original code by Marni,
 * rewritten to use driver by Sensirion, help from Brent Rubell.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SHT4X_H
#define DRV_SHT4X_H

#include "Adafruit_SHT4x.h"
#include "drvBase.h"

/*!
    @brief  Class that provides a driver interface for the SHT4X sensor.
*/
class drvSht4x : public drvBase {

public:
  /*!
      @brief    Constructor for a SHT4X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvSht4x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the SHT4X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _sht4x = new Adafruit_SHT4x();
    if (!_sht4x->begin())
      return false;

    _sht4x->setPrecision(SHT4X_HIGH_PRECISION);
    _sht4x->setHeater(SHT4X_NO_HEATER);

    return true;
  }

  /*!
      @brief    Reads the SHT4X's temperature and humidity in one measurement
                so both metrics in a read pass come from the same sample.
      @returns  True if the measurement succeeded, False otherwise.
  */
  bool ReadDevice() override { return _sht4x->getEvent(&_humidity, &_temp); }

  /*!
      @brief    Gets the SHT4X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadSensorData())
      return false;
    tempEvent->temperature = _temp.temperature;
    return true;
  }

  /*!
      @brief    Gets the SHT4X's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (!ReadSensorData())
      return false;
    humidEvent->relative_humidity = _humidity.relative_humidity;
    return true;
  }

protected:
  Adafruit_SHT4x *_sht4x;          ///< SHT4X object
  sensors_event_t _temp = {0};     ///< Cached temperature event
  sensors_event_t _humidity = {0}; ///< Cached humidity event
};
#endif // drvSht4x