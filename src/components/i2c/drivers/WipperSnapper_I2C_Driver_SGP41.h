/*!
 * @file WipperSnapper_I2C_Driver_SGP41.h
 *
 * Device driver for the SGP41 VOC/gas sensor.
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

#ifndef WipperSnapper_I2C_Driver_SGP41_H
#define WipperSnapper_I2C_Driver_SGP41_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SGP41.h>
#include <NOxGasIndexAlgorithm.h>
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>

#define SGP41_FASTTICK_INTERVAL_MS 1000 ///< Enforce ~1 Hz cadence
#define SGP41_CONDITIONING_TICKS 10     ///< Recommended warmup cycles
#define SGP41_VOC_LEARNING_MS 60000UL   ///< VOC index meaningful after ~60s
#define SGP41_NOX_LEARNING_MS 300000UL  ///< NOx index meaningful after ~300s

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP41 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SGP41 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP41 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SGP41(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _sgp41 = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SGP41 sensor driver.
                Cleans up and deallocates the underlying Adafruit_SGP41 object
                when the driver is destroyed.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_SGP41() {
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
  bool begin() {
    _sgp41 = new Adafruit_SGP41();
    if (!_sgp41 || !_sgp41->begin((uint8_t)_sensorAddress, _i2c)) {
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
    }

    _selfTestResult = _sgp41->executeSelfTest();

    // Initialize cached values
    _rawValue = 0;
    _rawNOxValue = 0;
    _vocIdx = 0;
    _noxIdx = 0;
    _conditioningTicks = 0;
    _algoStartMs = millis();
    _lastFastMs = millis() - SGP41_FASTTICK_INTERVAL_MS;
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
    if (!_sgp41)
      return false;
    rawEvent->data[0] = (float)_rawValue;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP41's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) override {
    if (!_sgp41)
      return false;
    // Note: VOC algorithm learning period is ~60 seconds from startup.
    // Values are valid for publishing immediately, but become meaningful
    // only after this warmup interval.
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
  bool getEventNOxIndex(sensors_event_t *noxIndexEvent) override {
    if (!_sgp41)
      return false;
    // Note: NOx algorithm learning period is ~300 seconds from startup.
    // Values are valid for publishing immediately, but become meaningful
    // only after this warmup interval.
    noxIndexEvent->nox_index = _noxIdx;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief  Performs background sampling for the SGP41.

              This method enforces a ~1 Hz cadence recommended by the sensor
              datasheet. On each call, it checks the elapsed time since the last
              poll using `millis()`. If at least SGP41_FASTTICK_INTERVAL_MS ms
              have passed, it reads a new raw value and VOC index from the
     sensor and caches them in `_rawValue` and `_vocIdx`.

              Cached results are later returned by `getEventRaw()` and
              `getEventVOCIndex()` without re-triggering I2C traffic.

      @note   Called automatically from
              `WipperSnapper_Component_I2C::update()` once per loop iteration.
              Must be non-blocking (no delays). The millis-based guard ensures
              the sensor is not over-polled.
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp41)
      return;
    if (!gasEnabled())
      return;

    uint32_t now = millis();
    if (now - _lastFastMs >= SGP41_FASTTICK_INTERVAL_MS) {
      uint16_t srawVoc = 0;
      uint16_t srawNox = 0;
      bool readOk = false;

      if (_conditioningTicks < SGP41_CONDITIONING_TICKS) {
        // Conditioning is part of expected SGP41 startup usage.
        // It warms up the VOC sensing path and seeds early baseline behavior.
        // We currently use library defaults (50% RH, 25C) because Wippersnapper
        // does not yet provide reference humidity/temperature feeds to this
        // driver. Future integration point: pass external RH/T references here.
        readOk = _sgp41->executeConditioning(&srawVoc);
        srawNox = 0;
        _conditioningTicks++;
      } else {
        // After conditioning, 1 Hz raw sampling is expected usage for SGP41.
        // This call supports RH/T compensation; we currently keep defaults.
        // Future integration point: call
        // measureRawSignals(&srawVoc, &srawNox, rh, tempC)
        // once reference feeds are available.
        readOk = _sgp41->measureRawSignals(&srawVoc, &srawNox);
      }

      if (readOk) {
        _rawValue = srawVoc;
        _rawNOxValue = srawNox;

        // Follow Adafruit_SGP41 gas_index example flow:
        // raw ticks -> Sensirion VOC/NOx gas index algorithms.
        _vocIdx = _vocAlgorithm.process((int32_t)srawVoc);
        _noxIdx = _noxAlgorithm.process((int32_t)srawNox);

        // Learning-time guidance (from example):
        // VOC becomes meaningful after ~1 minute, NOx after ~5 minutes.
        // Future integration point: expose a quality/status flag once
        // Wippersnapper signal schema has a per-reading readiness field.
      }

      _lastFastMs = now;
    }
  }

protected:
  Adafruit_SGP41 *_sgp41; ///< Pointer to SGP41 sensor object

  /**
   * Cached latest measurements from the sensor.
   * - _rawValue: raw VOC sensor output (ticks)
   * - _rawNOxValue: raw NOx sensor output (ticks)
   * - _vocIdx: calculated VOC Gas Index
   * - _noxIdx: calculated NOx Gas Index
   */
  uint16_t _rawValue = 0;             ///< Raw VOC sensor output (ticks)
  uint16_t _rawNOxValue = 0;          ///< Raw NOx sensor output (ticks)
  float _vocIdx = 0;                  ///< Calculated VOC Gas Index
  float _noxIdx = 0;                  ///< Calculated NOx Gas Index
  VOCGasIndexAlgorithm _vocAlgorithm; ///< VOC gas index state machine
  NOxGasIndexAlgorithm _noxAlgorithm; ///< NOx gas index state machine
  uint8_t _conditioningTicks = 0;     ///< Completed initial conditioning cycles
  uint32_t _algoStartMs = 0; ///< Timestamp when gas index learning began
  uint16_t _serialNumber[3] = {0, 0, 0}; ///< Optional serial number cache
  uint16_t _selfTestResult = 0;          ///< Optional self-test cache
  bool _hasSerial = false;  ///< True if serial number read succeeded
  uint32_t _lastFastMs = 0; ///< Last poll timestamp to enforce cadence

  /*******************************************************************************/
  /*!
      @brief  Returns whether VOC background sampling should be active.
      @return True if either VOC Index or raw value is configured to publish.
  */
  /*******************************************************************************/
  inline bool gasEnabled() {
    return (getSensorVOCIndexPeriod() > 0) || (getSensorNOxIndexPeriod() > 0) ||
           (getSensorRawPeriod() > 0);
  }
};

#endif // WipperSnapper_I2C_Driver_SGP41_H
