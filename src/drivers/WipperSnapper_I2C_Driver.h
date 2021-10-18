/*!
 * @file WipperSnapper_I2C_Driver.h
 *
 * Base implementation for I2C device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_H
#define WipperSnapper_I2C_Driver_H

#include "Wippersnapper.h"

/**************************************************************************/
/*!
    @brief  Base class for I2C Drivers.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an I2C sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver() {
    // TODO
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver() {
    // TODO
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the initialization status of an I2C driver.
      @returns  True if I2C driver is initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool getInitialized() { return isInitialized; }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a temperature sensor and converts
                the reading into the expected SI unit.
  */
  /*******************************************************************************/
  virtual void updateTemperature(float *temperature) {
    // no-op
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a humidity sensor and converts
                the reading into the expected SI unit.
  */
  /*******************************************************************************/
  virtual void updateHumidity(float *humidity) {
    // no-op
  }

  virtual bool getEnabledTemperatureSensor() {
    return _hasTempSensor;
  }

  virtual bool getEnabledHumidSensor() {
    return _hasHumidSensor;
  }

protected:
  bool isInitialized = false;
  uint16_t _sensorAddress; ///< The I2C device's unique I2C address
  bool _hasTempSensor = false;
  bool _hasHumidSensor = false;
};

#endif // WipperSnapper_I2C_Driver_H