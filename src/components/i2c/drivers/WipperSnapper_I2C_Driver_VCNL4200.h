/*!
 * @file WipperSnapper_I2C_Driver_VCNL4200.h
 *
 * Device driver for the VCNL4200 light + proximity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2024 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_VCNL4200_H
#define WipperSnapper_I2C_Driver_VCNL4200_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_VCNL4200.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4200 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VCNL4200 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4200 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VCNL4200(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4200 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VCNL4200() { delete _vcnl4200; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VCNL4200 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _vcnl4200 = new Adafruit_VCNL4200();
    bool status = false;
    // Attempt to initialize and configure VCNL4200
    if (!_vcnl4200->begin(_sensorAddress, _i2c)) {
      return false;
    }
    status = _vcnl4200->setALSshutdown(false);
    status &= _vcnl4200->setProxShutdown(false);
    status &= _vcnl4200->setProxHD(true); // 16bit instead of 12bit
    status &= _vcnl4200->setALSIntegrationTime(VCNL4200_ALS_IT_400MS);
    status &= _vcnl4200->setProxDuty(VCNL4200_PS_DUTY_1_160);
    status &= _vcnl4200->setProxLEDCurrent(VCNL4200_LED_I_200MA);
    status &= _vcnl4200->setProxIntegrationTime(VCNL4200_PS_IT_9T);
    return status;
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
    // Get sensor event populated in lux via AUTO integration and gain
    lightEvent->light = _vcnl4200->readALSdata();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the VCNL4200's proximity value into an event (no unit).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->data[0] = (float)_vcnl4200->readProxData();
    return true;
  }

protected:
  Adafruit_VCNL4200 *_vcnl4200; ///< Pointer to VCNL4200 light sensor object
};

#endif // WipperSnapper_I2C_Driver_VCNL4200