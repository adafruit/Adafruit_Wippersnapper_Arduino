/*!
 * @file drvIna219.h
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
#ifndef DRV_INA219_H
#define DRV_INA219_H

#include "drvBase.h"
#include <Adafruit_INA219.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA219 sensor.
*/
/**************************************************************************/
class drvIna219 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA219 sensor.
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
  drvIna219(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA219 sensor.
  */
  /*******************************************************************************/
  ~drvIna219() { delete _ina219; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA219 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ina219 = new Adafruit_INA219(_address);
    if (!_ina219->begin(_i2c))
      return false;

    // TODO: use setCalibration()

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

#endif // drvIna219