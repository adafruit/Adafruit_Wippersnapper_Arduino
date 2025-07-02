/*!
 * @file WipperSnapper_I2C_Driver_D6T1A.h
 *
 * Device driver for a D6T1A thermal sensor.
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

#ifndef WipperSnapper_I2C_Driver_D6T1A_H
#define WipperSnapper_I2C_Driver_D6T1A_H

#include "WipperSnapper_I2C_Driver.h"
#include <OmronD6T.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the D6T1A temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_D6T1A : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an D6T1A sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_D6T1A(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an D6T1A sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_D6T1A() { delete _d6t1a; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the D6T1A sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _d6t1a = new OmronD6T( OmronD6T.Model.D6T_1A, _i2c);
    // attempt to initialize D6T1A
    if (!_d6t1a->begin(_sensorAddress))
      return false;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the D6T1A's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _d6t1a->ambientTempC();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the D6T1A's object temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventObjectTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _d6t1a->objectTempC();
    return true;
  }


protected:
  OmronD6T *_d6t1a; ///< D6T1A  object
};

#endif // WipperSnapper_I2C_Driver_D6T1A