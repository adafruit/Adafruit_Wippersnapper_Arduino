/*!
 * @file drvTsl2591.h
 *
 * Device driver for the TSL2591 digital luminosity (light) sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_TSL2591_H
#define DRV_TSL2591_H

#include "drvBase.h"
#include <Adafruit_TSL2591.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a TSL2591 sensor.
*/
/**************************************************************************/
class drvTsl2591 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a TSL2591 sensor.
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
  drvTsl2591(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
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
      @brief    Destructor for an TSL2591 sensor.
  */
  /*******************************************************************************/
  ~drvTsl2591() { delete _tsl; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the TSL2591 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _tsl = new Adafruit_TSL2591(2591);
    // Attempt to initialize TSL2591
    if (!_tsl->begin(_i2c, TSL2591_ADDR))
      return false;

    // Configure TSL2591 sensor
    // Note: This driver uses the default configuration from
    // https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/examples/tsl2591/tsl2591.ino
    _tsl->setGain(TSL2591_GAIN_MED);                // 25x gain
    _tsl->setTiming(TSL2591_INTEGRATIONTIME_300MS); // 300ms integration time
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
    // Get sensor event
    _tsl->getEvent(lightEvent);

    // If lightEvent->light = 0 lux the sensor is probably saturated and no
    // reliable data could be generated! or if lightEvent->light is +/-
    // 4294967040 there was a float over/underflow
    if ((lightEvent->light == 0) | (lightEvent->light > 4294966000.0) |
        (lightEvent->light < -4294966000.0))
      return false;

    return true;
  }

protected:
  Adafruit_TSL2591 *_tsl; ///< Pointer to TSL2591 light sensor object
};

#endif // drvTsl2591