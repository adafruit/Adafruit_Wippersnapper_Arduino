/*!
 * @file drvMax44009.h
 *
 * Device driver for the MAX44009 ambient light sensor.
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
#ifndef DRV_MAX44009_H
#define DRV_MAX44009_H

#include "drvBase.h"
#include <Adafruit_MAX44009.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the MAX44009
            ambient light sensor.
*/
/**************************************************************************/
class drvMax44009 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MAX44009 sensor.
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
  drvMax44009(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a MAX44009 sensor.
  */
  /*******************************************************************************/
  ~drvMax44009() { delete _max44009; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MAX44009 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_max44009) {
      delete _max44009;
    }
    _max44009 = new Adafruit_MAX44009();
    if (!_max44009->begin((uint8_t)_address, _i2c))
      return false;
    // Explicitly set default (auto-ranging) mode so a future library
    // change cannot silently alter WipperSnapper behavior.
    // DEFAULT: Auto-ranging, measures every 800ms (lowest power)
    // CUSTOM SETTINGS CANDIDATES (not yet exposed via the v2 properties API):
    //  - Mode: auto-ranging (default, 800ms) vs continuous/manual.
    //  - Manual integration time and current-division ratio for bright/dim
    //    environments, if exposed by the library.
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
    if (_lastRead != 0 && (now - _lastRead < 1000))
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
      @brief    Gets the MAX44009's current ambient light reading in lux.
      @param    luxEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLux(sensors_event_t *luxEvent) {
    if (!_readSensor())
      return false;
    *luxEvent = _cachedLight;
    return true;
  }

protected:
  Adafruit_MAX44009 *_max44009 = nullptr; ///< Pointer to MAX44009 sensor object
  unsigned long _lastRead = 0;            ///< Last sensor read time
  sensors_event_t _cachedLight = {0};     ///< Cached light reading
};

#endif // DRV_MAX44009_H
