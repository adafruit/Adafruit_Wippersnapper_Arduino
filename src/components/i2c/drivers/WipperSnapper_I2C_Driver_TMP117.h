/*!
 * @file WipperSnapper_I2C_Driver_TMP117.h
 *
 * Device driver for the TMP117 Temperature sensor.
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
#ifndef WipperSnapper_I2C_Driver_TMP117_H
#define WipperSnapper_I2C_Driver_TMP117_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_TMP117.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a TMP117 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_TMP117 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a TMP117 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_TMP117(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an TMP117 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_TMP117() {
    // Called when a TMP117 component is deleted.
    delete _tmp117;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the TMP117 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _tmp117 = new Adafruit_TMP117();
    return _tmp117->begin((uint8_t)_sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the TMP117's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _tmp117->getEvent(tempEvent);
  }

protected:
  Adafruit_TMP117 *_tmp117; ///< Pointer to TMP117 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_TMP117