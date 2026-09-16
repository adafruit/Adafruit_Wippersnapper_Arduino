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
/// The gas-index algorithms output 0 while uptime <= 45s, i.e. for the first
/// 46 one-second samples (sensirion_gas_index_algorithm.c, INITIAL_BLACKOUT)
#define SGP41_BLACKOUT_SAMPLES 46
/// Datasheet 3.1: conditioning is recommended for 10s but "must not be exceeded
/// to avoid damage to the sensing material" - stop early under loop jitter
#define SGP41_CONDITIONING_MAX_MS 9000
#define SGP41_VOC_LEARNING_MS 60000UL  ///< VOC index meaningful after ~60s
#define SGP41_NOX_LEARNING_MS 300000UL ///< NOx index meaningful after ~300s

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
    // Conditioning and the gas-index algorithms need raw signals at a fixed
    // ~1 Hz, independent of the publish period - opt in to the controller's
    // fastTick() cadence.
    _fast_tick_ms = SGP41_FASTTICK_INTERVAL_MS;
    _tick_lead_ms = TICK_ALWAYS;
    _discard_samples = SGP41_BLACKOUT_SAMPLES;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SGP41 sensor driver. Turns the heater off
                and deallocates the underlying Adafruit_SGP41 object.
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
      @brief    Initializes the SGP41 sensor and begins I2C. Reads the serial
                number and runs the built-in self test for diagnostics, then
                resets the sampling state so conditioning starts from scratch.
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

    uint16_t serialNumber[3] = {0, 0, 0};
    _hasSerial = _sgp41->getSerialNumber(serialNumber);
    if (_hasSerial) {
      _serialNumber[0] = serialNumber[0];
      _serialNumber[1] = serialNumber[1];
      _serialNumber[2] = serialNumber[2];
      WS_DEBUG_PRINT("SGP41 serial: ");
      WS_DEBUG_PRINTHEX(_serialNumber[0]);
      WS_DEBUG_PRINTHEX(_serialNumber[1]);
      WS_DEBUG_PRINTHEX(_serialNumber[2]);
      WS_DEBUG_PRINTLN("");
    }

    // Self test result: 0 = all pixels pass; bits 0/1 flag VOC/NOx faults
    _selfTestResult = _sgp41->executeSelfTest();
    if (_selfTestResult != 0) {
      WS_DEBUG_PRINT("SGP41 self test reported a fault: 0x");
      WS_DEBUG_PRINTHEX(_selfTestResult);
      WS_DEBUG_PRINTLN("");
    }

    _rawValue = 0;
    _rawNOxValue = 0;
    _vocIdx = 0;
    _noxIdx = 0;
    _conditioningTicks = 0;
    return true;

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Humidity compensation: measureRawSignals(rh, tempC) from a paired
    //    RH/T sensor improves VOC/NOx accuracy (defaults to 50% RH, 25C).
    //  - Conditioning duration (number of warmup cycles before sampling).
  }

  /*******************************************************************************/
  /*!
      @brief    Background sampling for the SGP41, called by the controller
                every _fast_tick_ms. The first SGP41_CONDITIONING_TICKS cycles
                run the datasheet's conditioning command, which warms the
                sensing path; no samples are filed and the gas-index
                algorithms are not fed until conditioning completes (per the
                Sensirion reference usage). After that each tick takes one raw
                VOC/NOx measurement and runs both algorithms; the first
                SGP41_BLACKOUT_SAMPLES results are discarded by NewSample()
                while the algorithms are in their initial blackout. The cache
                is left untouched if a measurement fails.
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp41)
      return;

    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;

    if (_conditioningTicks < SGP41_CONDITIONING_TICKS) {
      // Conditioning is part of expected SGP41 startup usage.
      // It warms up the VOC sensing path and seeds early baseline behavior.
      ulong now = millis();
      if (_conditioningTicks == 0)
        _conditioning_start = now;
      if (now - _conditioning_start >= SGP41_CONDITIONING_MAX_MS) {
        // Ticks stretched past the datasheet's 10s ceiling: stop conditioning
        _conditioningTicks = SGP41_CONDITIONING_TICKS;
      } else {
        _sgp41->executeConditioning(&srawVoc);
        _conditioningTicks++;
        return;
      }
    }

    // After conditioning, 1 Hz raw sampling is expected usage for SGP41.
    if (_sgp41->measureRawSignals(&srawVoc, &srawNox)) {
      _rawValue = srawVoc;
      _rawNOxValue = srawNox;
      _vocIdx = _vocAlgorithm.process((int32_t)srawVoc);
      _noxIdx = _noxAlgorithm.process((int32_t)srawNox);
      NewSample();
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current raw unprocessed VOC value (cached
                from the most recent fastTick() sample).
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the raw value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_sgp41 || !AttemptRead())
      return false;
    rawEvent->data[0] = (float)_rawValue;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP41's current VOC reading. Not available until
                conditioning and the algorithm blackout have completed.
                Note: the VOC algorithm learning period is
                ~SGP41_VOC_LEARNING_MS from then; values are valid for
                publishing immediately, but become meaningful only after it.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal).
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    // A genuine post-blackout index is clamped >= 0.5; 0 means still learning
    if (!_sgp41 || !AttemptRead() || _vocIdx <= 0)
      return false;
    vocIndexEvent->voc_index = _vocIdx;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP41's current NOx reading. Not available until
                conditioning and the algorithm blackout have completed.
                Note: the NOx algorithm learning period is
                ~SGP41_NOX_LEARNING_MS from then; values are valid for
                publishing immediately, but become meaningful only after it.
      @param    noxIndexEvent
                  Adafruit Sensor event for NOx Index.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventNOxIndex(sensors_event_t *noxIndexEvent) {
    // A genuine post-blackout index is clamped >= 0.5; 0 means still learning
    if (!_sgp41 || !AttemptRead() || _noxIdx <= 0)
      return false;
    noxIndexEvent->nox_index = _noxIdx;
    return true;
  }

protected:
  Adafruit_SGP41 *_sgp41;             ///< Pointer to SGP41 sensor object
  uint16_t _rawValue = 0;             ///< Raw VOC sensor output (ticks)
  uint16_t _rawNOxValue = 0;          ///< Raw NOx sensor output (ticks)
  float _vocIdx = 0;                  ///< Calculated VOC Gas Index
  float _noxIdx = 0;                  ///< Calculated NOx Gas Index
  VOCGasIndexAlgorithm _vocAlgorithm; ///< VOC gas index state machine
  NOxGasIndexAlgorithm _noxAlgorithm; ///< NOx gas index state machine
  uint8_t _conditioningTicks = 0;     ///< Completed initial conditioning cycles
  ulong _conditioning_start = 0;      ///< millis() conditioning began
  uint16_t _serialNumber[3] = {0, 0, 0}; ///< Optional serial number cache
  uint16_t _selfTestResult = 0;          ///< Optional self-test cache
  bool _hasSerial = false; ///< True if serial number read succeeded
};

#endif // DRV_SGP41_H
