/*!
 * @file drvVncl4200.h
 *
 * Device driver for the VCNL4200 light + proximity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_VCNL4200_H
#define DRV_VCNL4200_H

#include "drvBase.h"
#include <Adafruit_VCNL4200.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4200 sensor.
*/
/**************************************************************************/
class drvVncl4200 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4200 sensor.
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
  drvVncl4200(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4200 sensor.
  */
  /*******************************************************************************/
  ~drvVncl4200() { delete _vcnl4200; }

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
    if (!_vcnl4200->begin(_address, _i2c)) {
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

#endif // DRV_VCNL4200_H