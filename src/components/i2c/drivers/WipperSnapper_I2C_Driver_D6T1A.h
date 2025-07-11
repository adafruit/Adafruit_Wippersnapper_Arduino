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

#include <OmronD6T.h>

#include "WipperSnapper_I2C_Driver.h"

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
    _d6t1a = new OmronD6T(OmronD6T::D6T_1A, _i2c);
    // attempt to initialize D6T1A
    if (!_d6t1a->begin(_sensorAddress))
      return false;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  /*******************************************************************************/
  bool HasBeenReadInLast200ms() {
    return _lastRead != 0 && millis() - _lastRead < 200;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    // dont read sensor more than once per 200ms
    if (HasBeenReadInLast200ms()) {
      return true;
    }

    _d6t1a->read();
    _deviceTemp = (float)_d6t1a->ambientTempC();
    _objectTemp = (float)_d6t1a->objectTempC(0, 0);
    _lastRead = millis();
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
    if (ReadSensorData() && _deviceTemp != NAN) {
      // if the sensor was read recently, return the cached temperature
      tempEvent->temperature = _deviceTemp;
      return true;
    }
    return false; // sensor not read recently, return false
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
    if (ReadSensorData() && _objectTemp != NAN) {
      // if the sensor was read recently, return the cached temperature
      tempEvent->temperature = _objectTemp;
      return true;
    }
    return false; // sensor not read recently, return false
  }

protected:
  float _deviceTemp = NAN;    ///< Device temperature in Celsius
  float _objectTemp = NAN;    ///< Object temperature in Celsius
  uint32_t _lastRead = 0;     ///< Last time the sensor was read in milliseconds
  OmronD6T *_d6t1a = nullptr; ///< D6T1A object
};

#endif // WipperSnapper_I2C_Driver_D6T1A