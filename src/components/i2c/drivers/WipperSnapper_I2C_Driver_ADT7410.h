/*!
 * @file WipperSnapper_I2C_Driver_ADT7410.h
 *
 * Device driver for the ADT7410 Temperature sensor.
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
#ifndef WipperSnapper_I2C_Driver_ADT7410_H
#define WipperSnapper_I2C_Driver_ADT7410_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_ADT7410.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a ADT7410 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_ADT7410 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a ADT7410 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_ADT7410(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an ADT7410 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_ADT7410() {
    // Called when a ADT7410 component is deleted.
    delete _ADT7410;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the ADT7410 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ADT7410 = new Adafruit_ADT7410();
    return _ADT7410->begin((uint8_t)_sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the ADT7410's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _ADT7410->readTempC();
    return true;
  }

protected:
  Adafruit_ADT7410 *_ADT7410; ///< Pointer to ADT7410 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_ADT7410