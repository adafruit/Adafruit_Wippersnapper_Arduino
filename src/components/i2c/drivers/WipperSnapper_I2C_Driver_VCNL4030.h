/*!
 * @file WipperSnapper_I2C_Driver_VCNL4030.h
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
#ifndef WipperSnapper_I2C_Driver_VCNL4030_H
#define WipperSnapper_I2C_Driver_VCNL4030_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_VCNL4030.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4030 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VCNL4030 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4030 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VCNL4030(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4030 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VCNL4030() { delete _vcnl4030; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VCNL4030 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    if (_vcnl4030)
      delete _vcnl4030;
    _vcnl4030 = new Adafruit_VCNL4030();
    if (!_vcnl4030->begin(_sensorAddress, _i2c))
      return false;

    // Enable and configure sensors, compound result catches any failure
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
      @brief    Performs a light sensor read using the Adafruit
                Unified Sensor API.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    lightEvent->light = _vcnl4030->readLux();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the VCNL4030's proximity value into an event (no unit).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->data[0] = (float)_vcnl4030->readProximity();
    return true;
  }

protected:
  Adafruit_VCNL4030 *_vcnl4030 = nullptr; ///< Pointer to VCNL4030 sensor object
};

#endif // WipperSnapper_I2C_Driver_VCNL4030_H
