/*!
 * @file WipperSnapper_I2C_Driver_LTR329.h
 *
 * Device driver for the LTR329 light sensor.
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
#ifndef WipperSnapper_I2C_Driver_LTR329_H
#define WipperSnapper_I2C_Driver_LTR329_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LTR329_LTR303.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LTR329 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LTR329 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LTR329 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LTR329(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LTR329 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LTR329() { delete _LTR329; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LTR329 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _LTR329 = new Adafruit_LTR329();
    // Attempt to initialize LTR329
    if (!_LTR329->begin(_i2c))
      return false;

    // Configure LTR329 sensor - Note: This uses the default configuration from
    // https://github.com/adafruit/Adafruit_LTR329_LTR303/blob/main/examples/ltr329_simpletest/ltr329_simpletest.ino
    _LTR329->setGain(LTR3XX_GAIN_2);
    _LTR329->setIntegrationTime(LTR3XX_INTEGTIME_100);
    _LTR329->setMeasurementRate(LTR3XX_MEASRATE_200);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the LTR329's ambient light level ([Visible+IR] - IR-only)
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!_LTR329->newDataAvailable())
      return false;

    uint16_t visible_plus_ir, infrared;
    _LTR329->readBothChannels(visible_plus_ir, infrared);
    lightEvent->light = visible_plus_ir - infrared;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the LTR329's infrared value into an event.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!_LTR329->newDataAvailable())
      return false;

    uint16_t visible_plus_ir, infrared;
    _LTR329->readBothChannels(visible_plus_ir, infrared);
    rawEvent->data[0] = (float)infrared;
    return true;
  }

protected:
  Adafruit_LTR329 *_LTR329; ///< Pointer to LTR329 light sensor object
};

#endif // WipperSnapper_I2C_Driver_LTR329