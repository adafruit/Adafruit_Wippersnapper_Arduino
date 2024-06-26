/*!
 * @file WipperSnapper_I2C_Driver_LTR329_LTR303.h
 *
 * Device driver for the LTR329 + LTR303 (329+interrupt) light sensors.
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
#ifndef WipperSnapper_I2C_Driver_LTR329_LTR303_H
#define WipperSnapper_I2C_Driver_LTR329_LTR303_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LTR329_LTR303.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LTR329/303 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LTR329_LTR303 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LTR329/303 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LTR329_LTR303(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LTR329/303 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LTR329_LTR303() { delete _LTR329; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LTR329/303 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _LTR329 = new Adafruit_LTR329();
    // Attempt to initialize LTR329
    if (!_LTR329->begin(_i2c))
      return false;

    // Configure LTR329 sensor - tested on a dull British day Oct'23 @7kLux
    // Matches similar lux value from LTR390 with default configuration.
    _LTR329->setGain(LTR3XX_GAIN_48);
    _LTR329->setIntegrationTime(LTR3XX_INTEGTIME_100);
    _LTR329->setMeasurementRate(LTR3XX_MEASRATE_100);
    _delayBetweenReads = 110; // 100ms measurement time + 10ms fudge-factor
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the LTR329's ambient light level ([Visible+IR] - IR-only)
      @param    lightEvent
                Light sensor reading.
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

    if (!_LTR329->newDataAvailable()) {
      delay(
          _delayBetweenReads); // Raw comes after Light event and needs new data
      if (!_LTR329->newDataAvailable())
        return false;
    }

    uint16_t visible_plus_ir, infrared;
    _LTR329->readBothChannels(visible_plus_ir, infrared);
    rawEvent->data[0] = (float)infrared;
    return true;
  }

protected:
  Adafruit_LTR329 *_LTR329; ///< Pointer to LTR329 light sensor object

  /**
   * @brief The delay between consecutive reads in milliseconds.
   */
  uint16_t _delayBetweenReads;
};

#endif // WipperSnapper_I2C_Driver_LTR329_LTR303