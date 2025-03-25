/*!
 * @file drvLc709203f.h
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
#ifndef DRV_LC709203F_H
#define DRV_LC709203F_H

#include "drvBase.h"
#include <Adafruit_LC709203F.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LC709203F sensor.
*/
/**************************************************************************/
class drvLc709203f : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LC709203F sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvLc709203f(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LC709203F sensor.
  */
  /*******************************************************************************/
  ~drvLc709203f() { delete _lc; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LC709203F sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _lc = new Adafruit_LC709203F();
    if (!_lc->begin(_i2c))
      return false;

    // Default settings from LC709203F demo:
    // https://github.com/adafruit/Adafruit_LC709203F/blob/master/examples/LC709203F_demo/LC709203F_demo.ino
    // NOTE: in the future, it would be nice if these able to be user-defined!
    if (!_lc->setThermistorB(3950))
      return false;
    if (!_lc->setPackSize(LC709203F_APA_500MAH))
      return false;
    if (!_lc->setAlarmVoltage(3.8))
      return false;

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
    unitlessPercentEvent->unitless_percent = _lc->cellPercent();
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] = wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
    _default_sensor_types[1] = wippersnapper_sensor_SensorType_SENSOR_TYPE_UNITLESS_PERCENT;
  }

protected:
  Adafruit_LC709203F *_lc; ///< Pointer to LC709203F sensor object
};

#endif // drvLc709203f