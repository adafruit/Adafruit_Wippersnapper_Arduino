/*!
 * @file WipperSnapper_I2C_Driver_PCT2075.h
 *
 * Device driver for the PCT2075 Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_PCT2075_H
#define WipperSnapper_I2C_Driver_PCT2075_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_PCT2075.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a PCT2075 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_PCT2075 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a PCT2075 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_PCT2075(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an PCT2075 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_PCT2075() {
    // Called when a PCT2075 component is deleted.
    delete _pct2075;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the PCT2075 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _pct2075 = new Adafruit_PCT2075();
    return _pct2075->begin((uint8_t)_sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the PCT2075's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _pct2075->getTemperature();
    return true;
  }

protected:
  Adafruit_PCT2075 *_pct2075; ///< Pointer to PCT2075 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_PCT2075