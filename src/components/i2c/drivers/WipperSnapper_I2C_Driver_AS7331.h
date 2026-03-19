/*!
 * @file WipperSnapper_I2C_Driver_AS7331.h
 *
 * Device driver for the AS7331 UV spectral sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
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
    @brief  Class that provides a driver interface for the AS7331 UV
            spectral sensor.
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

    // Configure the sensor for continuous measurement.
    // Power down to enter configuration state before setting parameters.
    _as7331->powerDown(true);
    _as7331->setGain(AS7331_GAIN_4X);
    _as7331->setIntegrationTime(AS7331_TIME_64MS);
    _as7331->setMeasurementMode(AS7331_MODE_CONT);
    _as7331->setClockFrequency(AS7331_CLOCK_1024MHZ);
    _as7331->setBreakTime(25); // 200us break time between measurements
    _as7331->setStandby(false);
    _as7331->setReadyPinOpenDrain(false); // push-pull READY pin output
    // Exit power-down and start continuous measurements
    _as7331->powerDown(false);

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the AS7331 UV-B channel and returns it as a raw sensor
                value. Also debug-logs UVA, UVB, and UVC counts to serial.
      @param    rawEvent
                Pointer to an Adafruit Unified Sensor event.
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
  uint16_t _uva = 0;                  ///< Cached UVA raw counts
  uint16_t _uvb = 0;                  ///< Cached UVB raw counts
  uint16_t _uvc = 0;                  ///< Cached UVC raw counts
  unsigned long _lastRead = 0;        ///< Timestamp of last sensor read, in ms

  /*******************************************************************************/
  /*!
      @brief    Reads all UV channels from the AS7331, with a 1-second cache
                to avoid redundant I2C reads within the same WipperSnapper
                polling cycle. Logs UVA, UVB, and UVC raw counts to serial.
      @returns  True if cached data is available or a fresh read succeeded,
                False otherwise.
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
