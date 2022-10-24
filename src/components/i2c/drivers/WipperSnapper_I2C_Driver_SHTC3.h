/*!
 * @file WipperSnapper_I2C_Driver_SHTC3.h
 *
 * Device driver for the SHTC3 Temperature and Humidity Sensor
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

#ifndef WipperSnapper_I2C_Driver_SHTC3_H
#define WipperSnapper_I2C_Driver_SHTC3_H

#include "WipperSnapper_I2C_Driver.h"
#include <SHTSensor.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SHTC3 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SHTC3 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SHTC3 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SHTC3(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SHTC3 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _shtc3 = new SHTSensor(SHTSensor::SHTC3);
    return _shtc3->init(*_i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHTC3's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // populate temp and humidity objects with fresh data
    if (!_shtc3->readSample())
      return false;
    tempEvent->temperature = _shtc3->getTemperature();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHTC3's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // populate temp and humidity objects with fresh data
    if (!_shtc3->readSample())
      return false;
    humidEvent->relative_humidity = _shtc3->getHumidity();
    return true;
  }

protected:
  SHTSensor *_shtc3; ///< SHTC3 object
};

#endif // WipperSnapper_I2C_Driver_SHTC3