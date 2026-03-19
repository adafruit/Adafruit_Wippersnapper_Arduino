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
    @brief  Class that provides a driver interface for an AS7331 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_AS7331 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an AS7331 sensor.
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
      @brief    Destructor for an AS7331 sensor.
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

    // Pin library defaults explicitly (from fulltest example and CREG
    // registers): CREG1 default=0xA6: gain=AS7331_GAIN_4X (bits[7:4]=0xA=10),
    //                     time=AS7331_TIME_64MS (bits[3:0]=0x6=6)
    // CREG3 default=0x50: mode=AS7331_MODE_CONT (bits[7:6]=01),
    //                     clock=AS7331_CLOCK_1024MHZ (bits[5:4]=00)
    // BREAK default=0x19=25: break time 25*8µs=200µs
    if (!_as7331->powerDown(true))
      return false;
    if (!_as7331->setGain(AS7331_GAIN_4X))
      return false;
    if (!_as7331->setIntegrationTime(AS7331_TIME_64MS))
      return false;
    if (!_as7331->setMeasurementMode(AS7331_MODE_CONT))
      return false;
    if (!_as7331->setClockFrequency(AS7331_CLOCK_1024MHZ))
      return false;
    if (!_as7331->setBreakTime(25))
      return false;
    if (!_as7331->powerDown(false))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the AS7331 UV-B value into a raw sensor event. Also
                debug-logs UVA, UVB, and UVC raw counts to serial.
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

  uint16_t _uva = 0;           ///< Cached UVA raw counts
  uint16_t _uvb = 0;           ///< Cached UVB raw counts
  uint16_t _uvc = 0;           ///< Cached UVC raw counts
  unsigned long _lastRead = 0; ///< Timestamp of last sensor read (ms)

  /*******************************************************************************/
  /*!
      @brief    Reads all three UV channels from the AS7331, caching results
                for up to 1 second to avoid redundant I2C reads when multiple
                getEvent*() calls occur per cycle. Debug-logs UVA, UVB, and
                UVC raw counts to serial on each fresh read.
      @returns  True if cached data is available or a fresh read succeeded.
  */
  /*******************************************************************************/
  bool _readSensor() {
    if (_lastRead != 0 && millis() - _lastRead < 1000)
      return true; // use cached values
    if (!_as7331->readAllUV(&_uva, &_uvb, &_uvc))
      return false;
    WS_DEBUG_PRINT("AS7331 UVA: ");
    WS_DEBUG_PRINT(_uva);
    WS_DEBUG_PRINT(", UVB: ");
    WS_DEBUG_PRINT(_uvb);
    WS_DEBUG_PRINT(", UVC: ");
    WS_DEBUG_PRINTLN(_uvc);
    _lastRead = millis();
    return true;
  }
};

#endif // WipperSnapper_I2C_Driver_AS7331_H
