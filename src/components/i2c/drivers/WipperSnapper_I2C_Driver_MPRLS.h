/*!
 * @file WipperSnapper_I2C_Driver_MPRLS.h
 *
 * Device driver for a MPRLS precision pressure sensor breakout.
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

#ifndef WipperSnapper_I2C_Driver_MPRLS_H
#define WipperSnapper_I2C_Driver_MPRLS_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MPRLS.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the MPRLS sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MPRLS : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MPRLS sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MPRLS(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MPRLS sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MPRLS() { delete _mprls; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MPRLS sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _mprls = new Adafruit_MPRLS();
    // attempt to initialize MPRLS
    if (!_mprls->begin(_sensorAddress, _i2c))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    pressureEvent->pressure = _mprls->readPressure();
    return pressureEvent->pressure != NAN;
  }

protected:
  Adafruit_MPRLS *_mprls; ///< MPRLS  object
};

#endif // WipperSnapper_I2C_Driver_MPRLS