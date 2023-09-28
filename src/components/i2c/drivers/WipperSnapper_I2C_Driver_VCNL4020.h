/*!
 * @file WipperSnapper_I2C_Driver_VCNL4020.h
 *
 * Device driver for the VCNL4020 light + proximity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_VCNL4020_H
#define WipperSnapper_I2C_Driver_VCNL4020_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_VCNL4020.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4020 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VCNL4020 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4020 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VCNL4020(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4020 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VCNL4020() { delete _vcnl4020; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VCNL4020 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _vcnl4020 = new Adafruit_VCNL4020();
    // Attempt to initialize and configure VCNL4020
    if (!_vcnl4020->begin(_sensorAddress, _i2c))
      return false;

    // Power saving: this could be lower, but for reliability 200mA(max, 50mA
    // min)
    _vcnl4020->setProximityLEDCurrent(VCNL4020_LED_CURRENT_200MA);
    _vcnl4020->setProximityLEDDutyCycle(VCNL4020_LED_DUTY_1_40);
    // This could be 1T integration time (default), but set to 8T for
    // reliability
    _vcnl4020->setProximityIntegrationTime(
        VCNL4020_PROXIMITY_INTEGRATION_TIME_8T);
    _vcnl4020->setProximityHighResolution(true); // 12bit or 16bit resolution
    _vcnl4020->setAmbientIntegrationTime(
        VCNL4020_AMBIENT_INTEGRATION_TIME_80MS);
    return true;
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
    lightEvent->light = _vcnl4020->getLux();

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the VCNL4020's proximity value into an event (no unit).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->data[0] (float)_vcnl4020->getProximity();
    return true;
  }

protected:
  Adafruit_VCNL4020 *_vcnl4020; ///< Pointer to VCNL4020 light sensor object
};

#endif // WipperSnapper_I2C_Driver_VCNL4020