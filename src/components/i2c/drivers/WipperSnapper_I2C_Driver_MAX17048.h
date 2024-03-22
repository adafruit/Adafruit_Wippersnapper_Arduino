/*!
 * @file WipperSnapper_I2C_Driver_MAX17048.h
 *
 * Device driver for the MAX17048/MAX17049 LiPoly / LiIon Fuel Gauge and Battery
 * Monitor
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
#ifndef WipperSnapper_I2C_Driver_MAX17048_H
#define WipperSnapper_I2C_Driver_MAX17048_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MAX1704X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a MAX17048 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MAX17048 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MAX17048 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MAX17048(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MAX17048 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MAX17048() { delete _maxlipo; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MAX17048 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _maxlipo = new Adafruit_MAX17048();
    if (!_maxlipo->begin(_i2c)) {
      return false;
    }
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
    voltageEvent->voltage = _maxlipo->cellVoltage();
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
    unitlessPercentEvent->unitless_percent = _maxlipo->cellPercent();
    return true;
  }

protected:
  Adafruit_MAX17048 *_maxlipo; ///< Pointer to MAX17048 sensor object
};

#endif // WipperSnapper_I2C_Driver_MAX17048