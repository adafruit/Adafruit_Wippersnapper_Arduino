/*!
 * @file WipperSnapper_I2C_Driver_VEML7700.h
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
#ifndef WipperSnapper_I2C_Driver_VEML7700_H
#define WipperSnapper_I2C_Driver_VEML7700_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_VEML7700.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VEML7700 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VEML7700 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VEML7700 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VEML7700(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VEML7700 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VEML7700() { delete _veml; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VEML7700 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
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

#endif // WipperSnapper_I2C_Driver_VEML7700