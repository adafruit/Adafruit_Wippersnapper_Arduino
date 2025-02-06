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

#include "drvBase.h"
#include <SHTSensor.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SHT4X sensor.
*/
/**************************************************************************/
class drvSht4x : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SHT4X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  drvSht4x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SHT4X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _sht4x = new SHTSensor(SHTSensor::SHT4X);
    if (!_sht4x->init(*_i2c))
      return false;

    // configure SHT4x sensor
    _sht4x->setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHT4X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // populate temp and humidity objects with fresh data
    if (!_sht4x->readSample())
      return false;
    tempEvent->temperature = _sht4x->getTemperature();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHT4X's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // populate temp and humidity objects with fresh data
    if (!_sht4x->readSample())
      return false;
    humidEvent->relative_humidity = _sht4x->getHumidity();
    return true;
  }

protected:
  SHTSensor *_sht4x; ///< SHT4X object
};

#endif // drvSht4x