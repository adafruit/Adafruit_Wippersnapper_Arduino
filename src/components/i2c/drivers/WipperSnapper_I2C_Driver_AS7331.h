/*!
 * @file WipperSnapper_I2C_Driver_AS7331.h
 *
 * Device driver for the AS7331 UV spectral sensor.
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
#ifndef WipperSnapper_I2C_Driver_AS7331_H
#define WipperSnapper_I2C_Driver_AS7331_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_AS7331.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the AS7331 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_AS7331 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a AS7331 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_AS7331(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a AS7331 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_AS7331() { delete _as7331; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AS7331 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _as7331 = new Adafruit_AS7331();
    if (!_as7331->begin(_i2c, (uint8_t)_sensorAddress))
      return false;
    // Sensor configuration defaults (from fulltest example and library
    // defaults):
    _as7331->powerDown(true);
    _as7331->setGain(AS7331_GAIN_4X);
    _as7331->setIntegrationTime(AS7331_TIME_64MS);
    _as7331->setMeasurementMode(AS7331_MODE_CONT);
    _as7331->setClockFrequency(AS7331_CLOCK_1024MHZ);
    _as7331->powerDown(false);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the AS7331's UV-B value into a raw sensor event.
                All three UV channels (UVA, UVB, UVC) are read and debug-logged
                on each sensor read cycle.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_readSensor())
      return false;
    rawEvent->data[0] = (float)_uvb;
    return true;
  }

protected:
  Adafruit_AS7331 *_as7331 = nullptr; ///< Pointer to AS7331 sensor object
  uint16_t _uva = 0;                  ///< Cached UVA raw count
  uint16_t _uvb = 0;                  ///< Cached UVB raw count
  uint16_t _uvc = 0;                  ///< Cached UVC raw count
  unsigned long _lastRead = 0;        ///< Timestamp of last sensor read

  /*******************************************************************************/
  /*!
      @brief    Reads sensor data, caching for 1 second to avoid redundant
                I2C reads when multiple getEvent*() calls occur per cycle.
                Logs all three UV channels (UVA, UVB, UVC) to serial on
                each fresh read.
      @returns  True if cached or fresh data is available, False otherwise.
  */
  /*******************************************************************************/
  bool _readSensor() {
    if (_lastRead != 0 && millis() - _lastRead < 1000)
      return true; // use cached values
    if (!_as7331->readAllUV(&_uva, &_uvb, &_uvc))
      return false;
    WS_DEBUG_PRINT("AS7331 UVA: ");
    WS_DEBUG_PRINT(_uva);
    WS_DEBUG_PRINT(" UVB: ");
    WS_DEBUG_PRINT(_uvb);
    WS_DEBUG_PRINT(" UVC: ");
    WS_DEBUG_PRINTLN(_uvc);
    _lastRead = millis();
    return true;
  }
};

#endif // WipperSnapper_I2C_Driver_AS7331_H
