/*!
 * @file drvSgp40.h
 *
 * Device driver for the SGP40 VOC/gas sensor.
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

#ifndef DRV_SGP40_H
#define DRV_SGP40_H

#include "drvBase.h"
#include <Adafruit_SGP40.h>
#include <Wire.h>

#define SGP40_FASTTICK_INTERVAL_MS 1000 ///< Enforce ~1 Hz sampling cadence

/*!
    @brief  Class that provides a driver interface for the SGP40 sensor.
*/
class drvSgp40 : public drvBase {
public:
  /*!
      @brief    Constructor for a SGP40 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvSgp40(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the SGP40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _sgp40 = new Adafruit_SGP40();
    if (!_sgp40->begin(_i2c)) {
      return false;
    }

    // TODO: update to use setCalibration() and pass in temp/humidity
    _lastFastMs = millis() - SGP40_FASTTICK_INTERVAL_MS;
    return true;
  }

  /*!
      @brief  Background sampling for the SGP40. measureVocIndex() runs the
              Sensirion VOC algorithm internally and expects to be called at
              ~1 Hz, independent of the device's publish period, so sampling is
              done here (called every loop) rather than in the getEvent*
              handlers. Non-blocking; the millis() guard enforces the cadence.
  */
  void fastTick() override {
    if (!_sgp40)
      return;
    uint32_t now = millis();
    if (now - _lastFastMs < SGP40_FASTTICK_INTERVAL_MS)
      return;
    _lastFastMs = now;
    _rawValue = _sgp40->measureRaw();
    _vocIdx = (int32_t)_sgp40->measureVocIndex();
  }

  /*!
      @brief    Gets the sensor's current raw unprocessed value (cached from
                the most recent fastTick() sample).
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the value was obtained successfully, False otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_sgp40)
      return false;
    rawEvent->data[0] = (float)_rawValue;
    return true;
  }

  /*!
      @brief    Gets the SGP40's current VOC reading (cached from the most
                recent fastTick() sample).
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    if (!_sgp40)
      return false;
    vocIndexEvent->voc_index = (float)_vocIdx;
    return true;
  }

protected:
  Adafruit_SGP40 *_sgp40;   ///< SGP40 driver object
  uint16_t _rawValue = 0;   ///< Cached raw sensor output (ticks)
  int32_t _vocIdx = 0;      ///< Cached VOC Index (signed, per datasheet)
  uint32_t _lastFastMs = 0; ///< Last fastTick sample time (1 Hz guard)
};

#endif // DRV_SGP40_H
