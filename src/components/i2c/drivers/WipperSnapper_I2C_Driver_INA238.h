/*!
 * @file WipperSnapper_I2C_Driver_INA238.h
 *
 * Device driver for the INA238 High-precision DC Current and Voltage Monitor
 * 16-bit ADC with ±0.1% gain error, ±5µV offset voltage
 * Higher precision version compared to INA237
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
#ifndef WipperSnapper_I2C_Driver_INA238_H
#define WipperSnapper_I2C_Driver_INA238_H

#include "WipperSnapper_I2C_Driver.h"

// Forward declaration
class Adafruit_INA238;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA238 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA238 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA238 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_INA238(TwoWire *i2c, uint16_t sensorAddress);

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA238 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_INA238();

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA238 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin();

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
  bool getEventVoltage(sensors_event_t *voltageEvent);

  /**
   * @brief   Get the current sensor event.
   *
   * @param   currentEvent  Pointer to the current sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventCurrent(sensors_event_t *currentEvent);

  /**
   * @brief   Get the Raw (power) sensor event.
   *
   * @param   powerEvent  Pointer to the power sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventRaw(sensors_event_t *powerEvent);

protected:
  Adafruit_INA238 *_ina238; ///< Pointer to INA238 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA238