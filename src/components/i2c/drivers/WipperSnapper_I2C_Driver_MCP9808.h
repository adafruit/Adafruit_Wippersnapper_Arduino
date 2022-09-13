/*!
 * @file WipperSnapper_I2C_Driver_MCP9808.h
 *
 * Device driver for the MCP9808 Temperature sensor.
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
#ifndef WipperSnapper_I2C_Driver_MCP9808_H
#define WipperSnapper_I2C_Driver_MCP9808_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MCP9808.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a MCP9808 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MCP9808 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MCP9808 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MCP9808(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MCP9808 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MCP9808() {
    // Called when a MCP9808 component is deleted.
    delete _mcp9808;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MCP9808 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _mcp9808 = new Adafruit_MCP9808();
    return _mcp9808->begin((uint8_t)_sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MCP9808's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _mcp9808->readTempC();
    return true;
  }

protected:
  Adafruit_MCP9808 *_mcp9808; ///< Pointer to MCP9808 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_MCP9808