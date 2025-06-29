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
#include <Adafruit_INA238.h>

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
  WipperSnapper_I2C_Driver_INA238(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA238 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_INA238() { delete _ina238; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA238 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ina238 = new Adafruit_INA238();
    if (!_ina238->begin(_sensorAddress, _i2c)) {
      WS_DEBUG_PRINTLN("INA238 failed to initialise!");
      return false;
    }

    // Configuration based on INA238 example sketch
    // Set default shunt resistance and maximum current
    // Default 0.015 ohm shunt, 10A max current
    _ina238->setShunt(0.015, 10.0);
    
    // Set averaging for better accuracy (16 samples)
    _ina238->setAveragingCount(INA2XX_COUNT_16);
    
    // Set conversion times as per example
    _ina238->setVoltageConversionTime(INA2XX_TIME_150_us);
    _ina238->setCurrentConversionTime(INA2XX_TIME_280_us);

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
    voltageEvent->voltage = _ina238->getBusVoltage_V();
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
    currentEvent->current = _ina238->getCurrent_mA();
    return true;
  }

  /**
   * @brief   Get the Raw (power) sensor event.
   *
   * @param   powerEvent  Pointer to the power sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventRaw(sensors_event_t *powerEvent) {
    powerEvent->data[0] = _ina238->getPower_mW();
    return true;
  }

protected:
  Adafruit_INA238 *_ina238 = nullptr; ///< Pointer to INA238 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA238