/*!
 * @file WipperSnapper_I2C_Driver_SGP40.h
 *
 * Device driver for the SGP40 VOC/gas sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_SGP40_H
#define WipperSnapper_I2C_Driver_SGP40_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SGP40.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP40 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SGP40 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP40 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SGP40(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _sgp40 = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SGP40 sensor driver.
                Cleans up and deallocates the underlying Adafruit_SGP40 object
                when the driver is destroyed.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_SGP40() override {
    if (_sgp40) {
      delete _sgp40;
      _sgp40 = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _sgp40 = new Adafruit_SGP40();
    if (!_sgp40 || !_sgp40->begin(_i2c)) {
      delete _sgp40;
      _sgp40 = nullptr;
      return false;
    }
    // Initialize cached values
    _rawValue = 0;
    _vocIdx = 0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current raw unprocessed value.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the raw value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) override {
    if (!_sgp40)
      return false;
    rawEvent->data[0] = (float)_rawValue;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP40's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) override {
    if (!_sgp40)
      return false;
    vocIndexEvent->voc_index = (uint16_t)_vocIdx;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Performs background sampling for the SGP40.
                single poll (no averaging) and cache
                results for getEventRaw()/getEventVOCIndex().
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp40)
      return;
    if (!vocEnabled())
      return;

    // Single poll and cache latest values (no cadence/averaging)
    _rawValue = _sgp40->measureRaw();
    _vocIdx = (uint16_t)_sgp40->measureVocIndex();
  }

protected:
  Adafruit_SGP40 *_sgp40; ///< SGP40

  /** Cached latest measurements (no averaging). */
  uint16_t _rawValue = 0;
  uint16_t _vocIdx = 0; ///< Index for the VOC (volatile organic compounds) sensor reading

  /*******************************************************************************/
  /*!
      @brief  Returns whether VOC background sampling should be active.
      @return True if either VOC Index or raw value is configured to publish.
  */
  /*******************************************************************************/
  inline bool vocEnabled() {
    return (getSensorVOCIndexPeriod() > 0) || (getSensorRawPeriod() > 0);
  }
};

#endif // WipperSnapper_I2C_Driver_SGP40_H
