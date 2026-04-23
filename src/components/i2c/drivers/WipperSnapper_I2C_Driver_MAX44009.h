/*!
 * @file WipperSnapper_I2C_Driver_MAX44009.h
 *
 * Device driver for the MAX44009 ambient light sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_MAX44009_H
#define WipperSnapper_I2C_Driver_MAX44009_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MAX44009.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the MAX44009
            ambient light sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MAX44009 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MAX44009 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MAX44009(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a MAX44009 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MAX44009() { delete _max44009; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MAX44009 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    if (_max44009) {
      delete _max44009;
    }
    _max44009 = new Adafruit_MAX44009();
    if (!_max44009->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    // Explicitly set default (auto-ranging) mode so a future library
    // change cannot silently alter WipperSnapper behavior.
    // DEFAULT: Auto-ranging, measures every 800ms (lowest power)
    _max44009->setMode(MAX44009_MODE_DEFAULT);
    if (!(_max44009->getMode() == MAX44009_MODE_DEFAULT)) {
      WS_DEBUG_PRINTLN("Failed to set MAX44009 mode!");
      return false;
    }
    _lastRead = 0; // ensure sensor is read on first update() call
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the MAX44009 sensor if not recently read.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool _readSensor() {
    unsigned long now = millis();
    if (_lastRead != 0 && (now - _lastRead < ONE_SECOND_IN_MILLIS))
      return true; // recently read, use cached value

    float lux = _max44009->readLux();
    if (isnan(lux))
      return false;

    _cachedLight.light = lux;
    _lastRead = now;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MAX44009's current ambient light reading.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!_readSensor())
      return false;
    *lightEvent = _cachedLight;
    return true;
  }

protected:
  Adafruit_MAX44009 *_max44009 = nullptr; ///< Pointer to MAX44009 sensor object
  unsigned long _lastRead = 0;            ///< Last sensor read time
  sensors_event_t _cachedLight = {0};     ///< Cached light reading
};

#endif // WipperSnapper_I2C_Driver_MAX44009_H
