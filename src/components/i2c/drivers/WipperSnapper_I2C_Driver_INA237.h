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
#include <Adafruit_INA237.h>

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
  WipperSnapper_I2C_Driver_INA237(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA237 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_INA237() { delete _ina237; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA237 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ina237 = new Adafruit_INA237();
    if (!_ina237->begin(_sensorAddress, _i2c)) {
      WS_DEBUG_PRINTLN("INA237 failed to initialise!");
      return false;
    }

    // Configuration based on INA237 example sketch
    // Set default shunt resistance and maximum current
    // Default 0.015 ohm shunt, 10A max current
    _ina237->setShunt(0.015, 10.0);
    
    // Set averaging for better accuracy (16 samples)
    _ina237->setAveragingCount(INA2XX_COUNT_16);
    
    // Set conversion times as per example
    _ina237->setVoltageConversionTime(INA2XX_TIME_150_us);
    _ina237->setCurrentConversionTime(INA2XX_TIME_280_us);

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
    voltageEvent->voltage = _ina237->getBusVoltage_V();
    return true;
  }

  /**
   * @brief   Get the current sensor event.
   *
   * @param   currentEvent  Pointer to the current sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventCurrent(sensors_event_t *currentEvent) {
    currentEvent->current = _ina237->getCurrent_mA();
    return true;
  }

  /**
   * @brief   Get the raw (power) sensor event.
   *
   * @param   powerEvent  Pointer to the power sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventRaw(sensors_event_t *powerEvent) {
    powerEvent->data[0] = _ina237->getPower_mW();
    return true;
  }

protected:
  Adafruit_INA237 *_ina237 = nullptr; ///< Pointer to INA237 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA237