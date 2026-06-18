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
    return _sgp30->begin(_i2c);

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Baseline calibration: setIAQBaseline(eCO2_base, TVOC_base) restores a
    //    stored baseline so the sensor reports accurate values immediately
    //    after boot instead of needing ~12h of burn-in. getIAQBaseline()
    //    persists it.
    //  - Humidity compensation: setHumidity(absolute_humidity_mg_m3) computed
    //    from a paired temperature/humidity sensor improves VOC/eCO2 accuracy.
    //  - Measurement interval: the SGP30 expects IAQmeasure() once per second
    //  to
    //    keep its dynamic baseline algorithm valid.
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP30's current equivalent/estimated CO2 value.
      @param    eco2Event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the eCO2 reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventECO2(sensors_event_t *eco2Event) {
    if (!_sgp30->IAQmeasure())
      return false;
    eco2Event->eCO2 = (float)_sgp30->eCO2;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP30's current Total VOC (TVOC) reading, in ppb.
      @param    tvocEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the TVOC reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventTVOC(sensors_event_t *tvocEvent) {
    if (!_sgp30->IAQmeasure())
      return false;
    tvocEvent->tvoc = (float)_sgp30->TVOC;
    return true;
  }

protected:
  Adafruit_SGP30 *_sgp30; ///< SGP30 driver object
};

#endif // DRV_SGP30_H
