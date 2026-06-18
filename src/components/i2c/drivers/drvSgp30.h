/*!
 * @file drvSgp30.h
 *
 * Device driver for the SGP30 VOC/eCO2 gas sensor.
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

#ifndef DRV_SGP30_H
#define DRV_SGP30_H

#include "drvBase.h"
#include <Adafruit_SGP30.h>
#include <Wire.h>

#define SGP30_FASTTICK_INTERVAL_MS 1000 ///< Enforce ~1 Hz sampling cadence

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP30 sensor.
*/
/**************************************************************************/
class drvSgp30 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP30 sensor.
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
  drvSgp30(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a SGP30 sensor.
  */
  /*******************************************************************************/
  ~drvSgp30() { delete _sgp30; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _sgp30 = new Adafruit_SGP30();
    if (!_sgp30->begin(_i2c))
      return false;
    _lastFastMs = millis() - SGP30_FASTTICK_INTERVAL_MS;
    return true;

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Baseline calibration: setIAQBaseline(eCO2_base, TVOC_base) restores a
    //    stored baseline so the sensor reports accurate values immediately
    //    after boot instead of needing ~12h of burn-in. getIAQBaseline()
    //    persists it.
    //  - Humidity compensation: setHumidity(absolute_humidity_mg_m3) computed
    //    from a paired temperature/humidity sensor improves VOC/eCO2 accuracy.
  }

  /*******************************************************************************/
  /*!
      @brief  Background sampling for the SGP30. The IAQ baseline algorithm
              requires IAQmeasure() to be called at ~1 Hz, independent of the
              device's publish period, so sampling is done here (called every
              loop) rather than in the getEvent* handlers. Non-blocking; the
              millis() guard enforces the 1 Hz cadence.
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_sgp30)
      return;
    uint32_t now = millis();
    if (now - _lastFastMs < SGP30_FASTTICK_INTERVAL_MS)
      return;
    _lastFastMs = now;
    if (_sgp30->IAQmeasure()) {
      _eco2 = (uint16_t)_sgp30->eCO2;
      _tvoc = (uint16_t)_sgp30->TVOC;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP30's current equivalent/estimated CO2 value (cached
                from the most recent fastTick() sample).
      @param    eco2Event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the eCO2 reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventECO2(sensors_event_t *eco2Event) {
    if (!_sgp30)
      return false;
    eco2Event->eCO2 = (float)_eco2;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP30's current Total VOC (TVOC) reading, in ppb
                (cached from the most recent fastTick() sample).
      @param    tvocEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the TVOC reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventTVOC(sensors_event_t *tvocEvent) {
    if (!_sgp30)
      return false;
    tvocEvent->tvoc = (float)_tvoc;
    return true;
  }

protected:
  Adafruit_SGP30 *_sgp30;   ///< SGP30 driver object
  uint16_t _eco2 = 0;       ///< Cached eCO2 reading, in ppm
  uint16_t _tvoc = 0;       ///< Cached TVOC reading, in ppb
  uint32_t _lastFastMs = 0; ///< Last fastTick sample time (1 Hz guard)
};

#endif // DRV_SGP30_H
