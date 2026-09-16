/*!
 * @file drvSgp40.h
 *
 * Device driver for the SGP40 VOC gas sensor.
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
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>

#define SGP40_FASTTICK_INTERVAL_MS 1000 ///< Enforce ~1 Hz sampling cadence

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP40 sensor.
*/
/**************************************************************************/
class drvSgp40 : public drvBase {
public:
  /*******************************************************************************/
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
  /*******************************************************************************/
  drvSgp40(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // The VOC gas-index algorithm expects a raw signal at ~1 Hz, independent
    // of the publish period - opt in to the controller's fastTick() cadence.
    _fast_tick_ms = SGP40_FASTTICK_INTERVAL_MS;
    _tick_lead_ms = TICK_ALWAYS;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a SGP40 sensor.
  */
  /*******************************************************************************/
  ~drvSgp40() { delete _sgp40; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _sgp40 = new Adafruit_SGP40();
    if (!_sgp40->begin(_i2c)) {
      return false;
    }
    _rawValue = 0;
    _vocIdx = 0;
    return true;

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Humidity compensation: measureRaw(temperature, humidity) from a paired
    //    RH/T sensor improves VOC accuracy (defaults to 25C, 50% RH).
  }

  /*******************************************************************************/
  /*!
      @brief    Background sampling for the SGP40, called by the controller
                every _fast_tick_ms. Takes one raw measurement, feeds it to the
                VOC gas-index algorithm (one heater cycle per tick, rather than
                a second measurement via measureVocIndex()) and caches both for
                the getEvent*() accessors. The cache is left untouched if the
                measurement fails.
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp40)
      return;
    // measureRaw() returns 0 on an I2C/CRC failure; a genuine SRAW is never 0
    uint16_t sraw = _sgp40->measureRaw();
    if (sraw == 0)
      return;
    _rawValue = sraw;
    _vocIdx = _vocAlgorithm.process((int32_t)sraw);
    NewSample();
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current raw unprocessed value (cached from
                the most recent fastTick() sample).
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the value was obtained successfully, False otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_sgp40 || !AttemptRead())
      return false;
    rawEvent->data[0] = (float)_rawValue;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP40's current VOC reading (cached from the most
                recent fastTick() sample). Note: the VOC algorithm learning
                period is ~60 seconds from startup; values are valid for
                publishing immediately but become meaningful only after it.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    if (!_sgp40 || !AttemptRead())
      return false;
    vocIndexEvent->voc_index = (float)_vocIdx;
    return true;
  }

protected:
  Adafruit_SGP40 *_sgp40 = nullptr;   ///< SGP40 driver object
  VOCGasIndexAlgorithm _vocAlgorithm; ///< VOC gas index state machine
  uint16_t _rawValue = 0;             ///< Cached raw sensor output (ticks)
  int32_t _vocIdx = 0; ///< Cached VOC Index (signed, per datasheet)
};

#endif // DRV_SGP40_H
