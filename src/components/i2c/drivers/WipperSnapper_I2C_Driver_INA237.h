/*!
 * @file WipperSnapper_I2C_Driver_INA237.h
 *
 * Device driver for the INA237 DC Current and Voltage Monitor
 * 16-bit ADC with ±0.3% gain error, ±50µV offset voltage
 * Cost-effective version, lower precision than INA238
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
#ifndef WipperSnapper_I2C_Driver_INA237_H
#define WipperSnapper_I2C_Driver_INA237_H

#include "WipperSnapper_I2C_Driver.h"
#include "Wippersnapper.h"

// Forward declaration
class Adafruit_INA237;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA237 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA237 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA237 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_INA237(TwoWire *i2c, uint16_t sensorAddress);

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA237 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_INA237();

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA237 sensor and begins I2C.
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
   * @brief   Get the raw (power) sensor event.
   *
   * @param   powerEvent  Pointer to the power sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventRaw(sensors_event_t *powerEvent);

protected:
  Adafruit_INA237 *_ina237; ///< Pointer to INA237 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA237