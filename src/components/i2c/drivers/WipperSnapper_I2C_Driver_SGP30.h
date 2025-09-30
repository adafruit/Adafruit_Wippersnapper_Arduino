#ifndef WipperSnapper_I2C_Driver_SGP30_H
#define WipperSnapper_I2C_Driver_SGP30_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SGP30.h>

#define SGP30_FASTTICK_INTERVAL_MS 1000 ///< Enforce ~1 Hz cadence

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
  ~WipperSnapper_I2C_Driver_SGP30() {
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
    _sgp30->IAQinit(); // start IAQ algorithm

    // Initialize cached values and cadence
    _eco2 = 0;
    _tvoc = 0;
    _lastFastMs = millis() - SGP30_FASTTICK_INTERVAL_MS;
    return true;
  }

/*******************************************************************************/
/*!
    @brief    Gets the most recently cached eCO2 reading.

              This value is updated in `fastTick()` at a ~1 Hz cadence
              and returned directly here without re-triggering an I2C
              transaction.

    @param    senseEvent
              Pointer to an Adafruit Sensor event that will be populated
              with the cached eCO2 value (in ppm).

    @returns  True if a cached value is available, False otherwise.
*/
/*******************************************************************************/
  bool getEventECO2(sensors_event_t *senseEvent) override {
    if (!_sgp30)
      return false;
    senseEvent->eCO2 = _eco2;
    return true;
  }

/*******************************************************************************/
/*!
    @brief    Gets the most recently cached TVOC reading.

              This value is updated in `fastTick()` at a ~1 Hz cadence
              and returned directly here without re-triggering an I2C
              transaction.

    @param    senseEvent
              Pointer to an Adafruit Sensor event that will be populated
              with the cached TVOC value (in ppb).

    @returns  True if a cached value is available, False otherwise.
*/
/*******************************************************************************/
  bool getEventTVOC(sensors_event_t *senseEvent) override {
    if (!_sgp30)
      return false;
    senseEvent->tvoc = _tvoc;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief  Performs background sampling for the SGP30.

              This method enforces a ~1 Hz cadence recommended by the sensor
              datasheet. On each call, it checks the elapsed time since the
              last poll using `millis()`. If at least
              SGP30_FASTTICK_INTERVAL_MS have passed, it performs a single
              IAQ measurement and caches the results in `_eco2` and `_tvoc`.

              Cached values are then returned by `getEventECO2()` and
              `getEventTVOC()` without re-triggering I2C traffic.

      @note   Called automatically from
              `WipperSnapper_Component_I2C::update()` once per loop iteration.
              Must be non-blocking (no delays). The millis-based guard ensures
              the sensor is not over-polled.
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp30)
      return;
    if (!iaqEnabled())
      return;

    uint32_t now = millis();
    if (now - _lastFastMs >= SGP30_FASTTICK_INTERVAL_MS) {
      if (_sgp30->IAQmeasure()) {
        _eco2 = (uint16_t)_sgp30->eCO2;
        _tvoc = (uint16_t)_sgp30->TVOC;
      }
      _lastFastMs = now;
    }
  }

protected:
  Adafruit_SGP30 *_sgp30; ///< Pointer to SGP30 sensor object

  /** Cached latest measurements (no averaging). */
  uint16_t _eco2 = 0; ///< eCO2, in ppm
  uint16_t _tvoc = 0; ///< TVOC, in ppb

  /** Timestamp of last poll to enforce 1 Hz cadence. */
  uint32_t _lastFastMs = 0;

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

#endif // WipperSnapper_I2C_Driver_SGP30_H
