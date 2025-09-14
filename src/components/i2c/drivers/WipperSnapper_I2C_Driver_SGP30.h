#ifndef WipperSnapper_I2C_Driver_SGP30_H
#define WipperSnapper_I2C_Driver_SGP30_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SGP30.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a SGP30 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SGP30 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP30 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SGP30(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _sgp30 = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SGP30 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_SGP30() override {
    // Called when a SGP30 component is deleted.
    if (_sgp30) {
      delete _sgp30;
      _sgp30 = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _sgp30 = new Adafruit_SGP30();
    if (!_sgp30->begin(_i2c)) {
      delete _sgp30; // avoid leak on init failure
      _sgp30 = nullptr;
      return false;
    }
    _sgp30->IAQinit();            // start IAQ algorithm
    _lastFastMs = millis();       // reset fast sampler
    _n = _eco2Sum = _tvocSum = 0; // clear accumulators
    return true;
  }

bool getEventECO2(sensors_event_t *senseEvent) override {
  if (!_sgp30) return false;
  if (_n > 0) {
    senseEvent->eCO2 = (uint16_t)(_eco2Sum / _n);
    _eco2Sum = 0; _tvocSum = 0; _n = 0;
    return true;
  }
  if (_sgp30->IAQmeasure()) {
    senseEvent->eCO2 = (uint16_t)_sgp30->eCO2;
    return true;
  }
  return false;
}

bool getEventTVOC(sensors_event_t *senseEvent) override {
  if (!_sgp30) return false;
  if (_n > 0) {
    senseEvent->tvoc = (uint16_t)(_tvocSum / _n);
    _eco2Sum = 0; _tvocSum = 0; _n = 0;
    return true;
  }
  if (_sgp30->IAQmeasure()) {
    senseEvent->tvoc = (uint16_t)_sgp30->TVOC;
    return true;
  }
  return false;
}

  void fastTick() override {
    if (!iaqEnabled())
      return; // nothing enabled, save cycles
    uint32_t now = millis();
    if (now - _lastFastMs >= 1000) { // ~1 Hz cadence
      if (_sgp30 && _sgp30->IAQmeasure()) {
        _eco2Sum += _sgp30->eCO2; // uint16_t in library
        _tvocSum += _sgp30->TVOC; // uint16_t in library
        _n++;
      }
      _lastFastMs = now;
    }
  }

protected:
  Adafruit_SGP30 *_sgp30; ///< Pointer to SGP30 sensor object

  /** Millis timestamp of last 1 Hz background read. */
  uint32_t _lastFastMs = 0;

  /** Number of samples accumulated since last publish. */
  uint32_t _n = 0;

  /** Running sum of eCO2 samples for averaging. */
  uint32_t _eco2Sum = 0;

  /** Running sum of TVOC samples for averaging. */
  uint32_t _tvocSum = 0;

  /*******************************************************************************/
  /*!
      @brief  Returns whether IAQ background sampling should be active.
      @return True if either eCO2 or TVOC metrics are configured to publish.
  */
  /*******************************************************************************/
  inline bool iaqEnabled() {
    // Enable IAQ background reads if either metric is requested
    return (getSensorECO2Period() > 0) || (getSensorTVOCPeriod() > 0);
  }
};

#endif // WipperSnapper_I2C_Driver_SGP30
