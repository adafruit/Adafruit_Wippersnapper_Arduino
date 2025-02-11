/*!
 * @file drvMprls.h
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

#ifndef DRV_MPRLS_H
#define DRV_MPRLS_H

#include "drvBase.h"
#include <Adafruit_MPRLS.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the MPRLS sensor.
*/
/**************************************************************************/
class drvMprls : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MPRLS sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvMprls(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel, const char* driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MPRLS sensor.
  */
  /*******************************************************************************/
  ~drvMprls() { delete _mprls; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MPRLS sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _mprls = new Adafruit_MPRLS();
    // attempt to initialize MPRLS
    return _mprls->begin(_address, _i2c);
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

#endif // drvMprls