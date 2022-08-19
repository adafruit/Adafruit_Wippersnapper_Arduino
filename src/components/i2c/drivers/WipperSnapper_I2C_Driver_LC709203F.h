/*!
 * @file WipperSnapper_I2C_Driver_LC709203F.h
 *
 * Device driver for the LC709203F LiPoly / LiIon Fuel Gauge and
 * Battery Monitor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_LC709203F_H
#define WipperSnapper_I2C_Driver_LC709203F_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LC709203F.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LC709203F sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LC709203F : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LC709203F sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LC709203F(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LC709203F sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LC709203F() { delete _lc; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LC709203F sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _lc = new Adafruit_LC709203F();
    if (!_lc->begin(_i2c))
      return false;

    // Default settings from LC709203F demo:
    // https://github.com/adafruit/Adafruit_LC709203F/blob/master/examples/LC709203F_demo/LC709203F_demo.ino
    // NOTE: in the future, it would be nice if these able to be user-defined!
    _lc->setThermistorB(3950);
    _lc->setPackSize(LC709203F_APA_500MAH);
    _lc->setAlarmVoltage(3.8);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a voltage sensor and converts the
                reading into the expected SI unit.
      @param    voltageEvent
                voltage sensor reading, in volts.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVoltage(sensors_event_t *voltageEvent) {
    voltageEvent->voltage = _lc->cellVoltage();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a sensor's unitless % reading and
                converts the reading into the expected SI unit.
      @param    unitlessPercentEvent
                unitless % sensor reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventUnitlessPercent(sensors_event_t *unitlessPercentEvent) {
    unitlessPercentEvent->data[0] = _lc->cellPercent();
    return true;
  }

protected:
  Adafruit_LC709203F *_lc; ///< Pointer to LC709203F sensor object
};

#endif // WipperSnapper_I2C_Driver_LC709203F