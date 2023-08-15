/*!
 * @file WipperSnapper_I2C_Driver_MPL115A2.h
 *
 * Device driver for a MPL115A2 pressure sensor breakout.
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

#ifndef WipperSnapper_I2C_Driver_MPL115A2_H
#define WipperSnapper_I2C_Driver_MPL115A2_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MPL115A2.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the MPL115A2 temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MPL115A2 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MPL115A2 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MPL115A2(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MPL115A2 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MPL115A2() { delete _mpl115a2; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MPL115A2 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _mpl115a2 = new Adafruit_MPL115A2();
    // attempt to initialize MPL115A2
    if (!_mpl115a2->begin(_sensorAddress, _i2c))
      return false;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MPL115A2's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _mpl115a2->getTemperature();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit (hPa).
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    pressureEvent->pressure = _mpl115a2->getPressure() * 10;
    return true;
  }

protected:
  Adafruit_MPL115A2 *_mpl115a2; ///< MPL115A2  object
};

#endif // WipperSnapper_I2C_Driver_MPL115A2