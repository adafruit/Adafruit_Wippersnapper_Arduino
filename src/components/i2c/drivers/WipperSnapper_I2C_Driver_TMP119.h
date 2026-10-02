/*!
 * @file WipperSnapper_I2C_Driver_TMP119.h
 *
 * Device driver for the TMP119 high-accuracy temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_TMP119_H
#define WipperSnapper_I2C_Driver_TMP119_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_TMP119.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the TMP119 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_TMP119 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a TMP119 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_TMP119(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a TMP119 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_TMP119() { delete _tmp119; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the TMP119 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _tmp119 = new Adafruit_TMP119();
    if (!_tmp119->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    // Explicit defaults so library changes don't alter WipperSnapper behavior.
    if (!_tmp119->setMeasurementMode(TMP117_MODE_CONTINUOUS) ||
        !_tmp119->setAveragedSampleCount(TMP117_AVERAGE_8X))
      WS_DEBUG_PRINTLN("Failed to reconfigure TMP119 - continuing");
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the TMP119's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_readSensor())
      return false;
    *tempEvent = _cachedTemp;
    return true;
  }

protected:
  Adafruit_TMP119 *_tmp119 = nullptr; ///< TMP119 driver object
  unsigned long _lastRead = 0;        ///< Last sensor read time in ms
  sensors_event_t _cachedTemp = {0};  ///< Cached temperature event

  /*******************************************************************************/
  /*!
      @brief    Reads the TMP119 sensor data, caching the result so only
                the first call per cycle performs the I2C transaction.
      @returns  True if sensor data is available, False otherwise.
  */
  /*******************************************************************************/
  bool _readSensor() {
    if (_lastRead != 0 && millis() - _lastRead < ONE_SECOND_IN_MILLIS)
      return true; // use cached value
    if (!_tmp119->getEvent(&_cachedTemp))
      return false;
    _lastRead = millis();
    return true;
  }
};

#endif // WipperSnapper_I2C_Driver_TMP119_H
