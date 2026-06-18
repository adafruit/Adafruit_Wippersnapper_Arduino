/*!
 * @file drvVcnl4030.h
 *
 * Device driver for the VCNL4030 proximity + ambient light sensor.
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
#ifndef DRV_VCNL4030_H
#define DRV_VCNL4030_H

#include "drvBase.h"
#include <Adafruit_VCNL4030.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4030 sensor.
*/
/**************************************************************************/
class drvVcnl4030 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4030 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvVcnl4030(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4030 sensor.
  */
  /*******************************************************************************/
  ~drvVcnl4030() { delete _vcnl4030; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VCNL4030 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_vcnl4030) {
      delete _vcnl4030;
    }
    _vcnl4030 = new Adafruit_VCNL4030();
    if (!_vcnl4030->begin(_address, _i2c))
      return false;

    // Enable and configure sensors, compound result catches any failure.
    // CUSTOM SETTINGS CANDIDATES (currently hard-coded defaults below; not yet
    // exposed via the v2 properties API):
    //  - ALS integration time (setALSIntegrationTime): longer = higher lux
    //    resolution, slower update.
    //  - Proximity LED current (setProxLEDCurrent): higher = longer range.
    //  - Proximity duty cycle / integration time / 16-bit resolution.
    bool result = _vcnl4030->enableALS(true);
    result = _vcnl4030->enableProx(true) && result;
    result = _vcnl4030->setProxLEDCurrent(VCNL4030_PROX_LED_200MA) && result;
    result = _vcnl4030->setProxDuty(VCNL4030_PROX_DUTY_40) && result;
    result = _vcnl4030->setProxIntegrationTime(VCNL4030_PROX_IT_8T) && result;
    result = _vcnl4030->setProxResolution16Bit(true) && result;
    result = _vcnl4030->setALSIntegrationTime(VCNL4030_ALS_IT_100MS) && result;

    return result;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the VCNL4030 sensor data and caches the results.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    unsigned long now = millis();
    if (_lastRead != 0 && (now - _lastRead < 1000))
      return true;

    _cachedLight.light = _vcnl4030->readLux();
    _cachedProximity.data[0] = (float)_vcnl4030->readProximity();
    _lastRead = now;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VCNL4030's current light reading in lux.
      @param    lightEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the light reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!ReadSensorData())
      return false;

    *lightEvent = _cachedLight;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VCNL4030's current proximity reading.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    if (!ReadSensorData())
      return false;

    *proximityEvent = _cachedProximity;
    return true;
  }

protected:
  Adafruit_VCNL4030 *_vcnl4030 = nullptr; ///< Pointer to VCNL4030 sensor object
  unsigned long _lastRead = 0;            ///< Last sensor read time
  sensors_event_t _cachedLight = {0};     ///< Cached light reading
  sensors_event_t _cachedProximity = {0}; ///< Cached proximity reading
};

#endif // DRV_VCNL4030_H
