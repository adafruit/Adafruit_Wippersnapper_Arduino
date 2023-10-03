/*!
 * @file WipperSnapper_I2C_Driver_LTR390.h
 *
 * Device driver for the LTR390 light sensor.
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
#ifndef WipperSnapper_I2C_Driver_LTR390_H
#define WipperSnapper_I2C_Driver_LTR390_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LTR390.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LTR390 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LTR390 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LTR390 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LTR390(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LTR390 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LTR390() { delete _ltr390; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LTR390 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ltr390 = new Adafruit_LTR390();
    // Attempt to initialize LTR390
    if (!_ltr390->begin(_i2c))
      return false;

    // Configure LTR390 sensor
    // Note: This driver uses the default configuration from
    // https://github.com/adafruit/Adafruit_LTR390/blob/master/examples/ltr390_test/ltr390_test.ino
    _ltr390->setMode(LTR390_MODE_ALS);
    _ltr390->setGain(LTR390_GAIN_3);
    _ltr390->setResolution(LTR390_RESOLUTION_16BIT);
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
    if (_ltr390->getMode() != LTR390_MODE_ALS) {
      _ltr390->setMode(LTR390_MODE_ALS);
      delay(100);
    }

    if (!_ltr390->newDataAvailable())
      return false;

    lightEvent->light = _ltr390->readALS();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the LTR390's UV value into an event.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (_ltr390->getMode() != LTR390_MODE_UVS) {
      _ltr390->setMode(LTR390_MODE_UVS);
      delay(100);
    }

    if (!_ltr390->newDataAvailable())
      return false;

    rawEvent->data[0] = (float)_ltr390->readUVS();
    return true;
  }

protected:
  Adafruit_LTR390 *_ltr390; ///< Pointer to LTR390 light sensor object
};

#endif // WipperSnapper_I2C_Driver_LTR390