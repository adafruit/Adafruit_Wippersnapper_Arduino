/*!
 * @file drvMax1704x.h
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
#ifndef DRV_MAX1704x_H
#define DRV_MAX1704x_H

#include "drvBase.h"
#include <Adafruit_MAX1704X.h>

/*!
    @brief  Class that provides a driver interface for a MAX17048 sensor.
*/
class drvMax1704x : public drvBase {
public:
  /*!
      @brief    Constructor for a MAX17048 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvMax1704x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an MAX17048 sensor.
  */
  ~drvMax1704x() { delete _maxlipo; }

  /*!
      @brief    Initializes the MAX17048 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _maxlipo = new Adafruit_MAX17048();
    return _maxlipo->begin(_i2c);
  }

  /*!
      @brief    Reads a voltage sensor and converts the
                reading into the expected SI unit.
      @param    voltageEvent
                voltage sensor reading, in volts.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventVoltage(sensors_event_t *voltageEvent) {
    voltageEvent->voltage = _maxlipo->cellVoltage();
    return true;
  }

  /*!
      @brief    Reads a sensor's unitless % reading and
                converts the reading into the expected SI unit.
      @param    unitlessPercentEvent
                unitless % sensor reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventUnitlessPercent(sensors_event_t *unitlessPercentEvent) {
    unitlessPercentEvent->unitless_percent = _maxlipo->cellPercent();
    return true;
  }

protected:
  Adafruit_MAX17048 *_maxlipo; ///< Pointer to MAX17048 sensor object
};

#endif // drvMax1704x