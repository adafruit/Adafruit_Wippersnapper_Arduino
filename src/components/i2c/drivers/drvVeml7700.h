/*!
 * @file drvVeml7700.h
 *
 * Device driver for the VEML7700 digital luminosity (light) sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_VEML770_H
#define DRV_VEML770_H

#include "drvBase.h"
#include <Adafruit_VEML7700.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VEML7700 sensor.
*/
/**************************************************************************/
class drvVeml7700 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VEML7700 sensor.
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
  drvVeml7700(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VEML7700 sensor.
  */
  /*******************************************************************************/
  ~drvVeml7700() { delete _veml; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VEML7700 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _veml = new Adafruit_VEML7700();
    // Attempt to initialize and configure VEML7700
    return _veml->begin(_i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a light sensor read using the Adafruit
                Unified Sensor API. Always uses VEML_LUX_AUTO,
                controlling sensor integration time and gain.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    // Get sensor event populated in lux via AUTO integration and gain
    lightEvent->light = _veml->readLux(VEML_LUX_AUTO);

    return true;
  }

protected:
  Adafruit_VEML7700 *_veml; ///< Pointer to VEML7700 light sensor object
};

#endif // drvVeml7700