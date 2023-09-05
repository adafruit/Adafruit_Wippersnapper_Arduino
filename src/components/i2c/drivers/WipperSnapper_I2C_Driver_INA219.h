/*!
 * @file WipperSnapper_I2C_Driver_INA219.h
 *
 * Device driver for the INA219 High-side DC Current and Voltage Monitor
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
#ifndef WipperSnapper_I2C_Driver_INA219_H
#define WipperSnapper_I2C_Driver_INA219_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_INA219.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA219 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA219 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA219 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_INA219(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA219 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_INA219() { delete _ina219; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA219 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ina219 = new Adafruit_INA219(_sensorAddress);
    if (!_ina219->begin(_i2c))
      return false;

    // TODO: When parameters or dual input/output or sensor configuration added
    //  To use a slightly lower 32V, 1A range (higher precision on amps):
    // ina219.setCalibration_32V_1A();
    //  Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    // ina219.setCalibration_16V_400mA();

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
    float shuntvoltage_mV = _ina219->getShuntVoltage_mV();
    float busvoltage_V = _ina219->getBusVoltage_V();

    // Compute load voltage
    float loadvoltage = busvoltage_V + (shuntvoltage_mV / 1000);
    voltageEvent->voltage = loadvoltage;
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
    float current_mA = _ina219->getCurrent_mA();
    currentEvent->current = current_mA;
    return true;
  }

protected:
  Adafruit_INA219 *_ina219; ///< Pointer to INA219 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA219