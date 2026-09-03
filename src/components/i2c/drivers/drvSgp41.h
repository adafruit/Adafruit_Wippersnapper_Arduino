/*!
 * @file drvSgp41.h
 *
 * Device driver for the SGP41 VOC/NOx gas sensor.
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

#ifndef DRV_SGP41_H
#define DRV_SGP41_H

#include "drvBase.h"
#include <Adafruit_SGP41.h>
#include <NOxGasIndexAlgorithm.h>
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>

#define SGP41_FASTTICK_INTERVAL_MS 1000 ///< Enforce ~1 Hz sampling cadence
#define SGP41_CONDITIONING_TICKS 10     ///< Recommended warmup cycles

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP41 sensor.
*/
/**************************************************************************/
class drvSgp41 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP41 sensor.
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
  drvSgp41(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _sgp41 = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SGP41 sensor driver.
  */
  /*******************************************************************************/
  ~drvSgp41() {
    if (_sgp41) {
      _sgp41->turnHeaterOff();
      delete _sgp41;
      _sgp41 = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP41 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _sgp41 = new Adafruit_SGP41();
    if (!_sgp41 || !_sgp41->begin((uint8_t)_address, _i2c)) {
      delete _sgp41;
      _sgp41 = nullptr;
      return false;
    }

    _sgp41->softReset();
    _rawValue = 0;
    _vocIdx = 0;
    _noxIdx = 0;
    _conditioningTicks = 0;
    _lastFastMs = millis() - SGP41_FASTTICK_INTERVAL_MS;
    return true;

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Humidity compensation: measureRawSignals(rh, tempC) from a paired
    //    RH/T sensor improves VOC/NOx accuracy (defaults to 50% RH, 25C).
    //  - Conditioning duration (number of warmup cycles before sampling).
  }

  /*******************************************************************************/
  /*!
      @brief  Background sampling for the SGP41. The Sensirion gas-index
              algorithm and the initial conditioning require raw signals at a
              fixed ~1 Hz cadence, independent of how often VOC/NOx/raw are
              published, so the work is done here (called every loop) rather
              than in the getEvent* handlers. Non-blocking; the millis() guard
              enforces the 1 Hz cadence.
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp41)
      return;

    uint32_t now = millis();
    if (now - _lastFastMs < SGP41_FASTTICK_INTERVAL_MS)
      return;
    _lastFastMs = now;

    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;
    bool readOk = false;

    if (_conditioningTicks < SGP41_CONDITIONING_TICKS) {
      // Conditioning warms up the VOC sensing path and seeds early baseline.
      readOk = _sgp41->executeConditioning(&srawVoc);
      srawNox = 0;
      _conditioningTicks++;
    } else {
      readOk = _sgp41->measureRawSignals(&srawVoc, &srawNox);
    }

    if (readOk) {
      _rawValue = srawVoc;
      // raw ticks -> Sensirion VOC/NOx gas index algorithms
      _vocIdx = _vocAlgorithm.process((int32_t)srawVoc);
      _noxIdx = _noxAlgorithm.process((int32_t)srawNox);
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current raw unprocessed value (cached from
                the most recent fastTick() sample).
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the raw value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_sgp41)
      return false;
    rawEvent->data[0] = (float)_rawValue;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP41's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal).
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    if (!_sgp41)
      return false;
    // Note: VOC algorithm learning period is ~60 seconds from startup.
    vocIndexEvent->voc_index = _vocIdx;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP41's current NOx reading.
      @param    noxIndexEvent
                  Adafruit Sensor event for NOx Index.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventNOxIndex(sensors_event_t *noxIndexEvent) {
    if (!_sgp41)
      return false;
    // Note: NOx algorithm learning period is ~300 seconds from startup.
    noxIndexEvent->nox_index = _noxIdx;
    return true;
  }

protected:
  Adafruit_SGP41 *_sgp41;             ///< Pointer to SGP41 sensor object
  uint16_t _rawValue = 0;             ///< Raw VOC sensor output (ticks)
  float _vocIdx = 0;                  ///< Calculated VOC Gas Index
  float _noxIdx = 0;                  ///< Calculated NOx Gas Index
  VOCGasIndexAlgorithm _vocAlgorithm; ///< VOC gas index state machine
  NOxGasIndexAlgorithm _noxAlgorithm; ///< NOx gas index state machine
  uint8_t _conditioningTicks = 0;     ///< Completed initial conditioning cycles
  uint32_t _lastFastMs = 0; ///< Last fastTick sample time (1 Hz guard)
};

#endif // DRV_SGP41_H
