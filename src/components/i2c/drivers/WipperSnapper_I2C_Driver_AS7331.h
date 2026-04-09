/*!
 * @file WipperSnapper_I2C_Driver_AS7331.h
 *
 * Device driver for the AS7331 UV spectral sensor (UVA/UVB/UVC).
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
#ifndef WipperSnapper_I2C_Driver_AS7331_H
#define WipperSnapper_I2C_Driver_AS7331_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_AS7331.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for an AS7331 UV sensor.
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

    bool setupSuccess = true;
    // Configure sensor — must power down before changing settings
    setupSuccess &= _as7331->powerDown(true);
    if (!setupSuccess)
      return false;
    // Explicitly set defaults to pin behavior against library changes
    setupSuccess &= _as7331->setGain(AS7331_GAIN_4X);
    setupSuccess &= _as7331->setIntegrationTime(AS7331_TIME_64MS);
    setupSuccess &= _as7331->setMeasurementMode(AS7331_MODE_CONT);
    setupSuccess &= _as7331->setClockFrequency(AS7331_CLOCK_1024MHZ);
    setupSuccess &= _as7331->setBreakTime(25); // 200us
    setupSuccess &= _as7331->setStandby(false);

    // Start continuous measurements
    setupSuccess &= _as7331->powerDown(false);

    return setupSuccess;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AS7331's current UVB reading in uW/cm2 as raw event.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_readSensor())
      return false;

    rawEvent->data[0] = _cachedUVB;
    return true;
  }

protected:
  Adafruit_AS7331 *_as7331 = nullptr; ///< Pointer to AS7331 sensor object
  unsigned long _lastRead = 0;        ///< Last sensor read time
  float _cachedUVB = 0;               ///< Cached UVB reading in uW/cm2

  /*******************************************************************************/
  /*!
      @brief    Reads all UV channels from the AS7331, caches UVB, and
                prints UVA/UVB/UVC as debug output.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool _readSensor() {
    unsigned long now = millis();
    if (_lastRead != 0 && (now - _lastRead < 1000))
      return true;

    float uva, uvb, uvc;
    if (!_as7331->readAllUV_uWcm2(&uva, &uvb, &uvc))
      return false;

    _cachedUVB = uvb;
    _lastRead = now;

    WS_DEBUG_PRINT("AS7331 UVA: ");
    WS_DEBUG_PRINTVAR(uva);
    WS_DEBUG_PRINT(" UVB: ");
    WS_DEBUG_PRINTVAR(uvb);
    WS_DEBUG_PRINT(" UVC: ");
    WS_DEBUG_PRINTLNVAR(uvc);

    return true;
  }
};

#endif // WipperSnapper_I2C_Driver_AS7331_H
