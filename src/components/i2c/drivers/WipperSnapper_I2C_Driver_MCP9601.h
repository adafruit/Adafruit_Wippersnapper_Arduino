/*!
 * @file WipperSnapper_I2C_Driver_MCP9601.h
 *
 * Device driver for the MCP9601 Temperature sensor.
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
#ifndef WipperSnapper_I2C_Driver_MCP9601_H
#define WipperSnapper_I2C_Driver_MCP9601_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MCP9601.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a MCP9601 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MCP9601 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MCP9601 sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MCP9601(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    // Called when a MCP9601 component is created
    setI2CAddress(sensorAddress); // sets the driver's I2C address
    _MCP9601 = new Adafruit_MCP9601();
    _isInitialized = _MCP9601->begin();

    if (_isInitialized) {
      _MCP9601->setADCresolution(MCP9600_ADCRESOLUTION_18);
      _MCP9601->setThermocoupleType(MCP9600_TYPE_K);
      _MCP9601->setFilterCoefficient(3);
      _MCP9601->enable(true);
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MCP9601 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MCP9601() {
    // Called when a MCP9601 component is deleted.
    delete _MCP9601;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MCP9601's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    uint8_t status = mcp.getStatus();
    if (status & MCP9601_STATUS_OPENCIRCUIT) { 
      return; // don't continue, since there's no thermocouple
    }
    if (status & MCP9601_STATUS_SHORTCIRCUIT) { 
      return; // don't continue, since the sensor is not working
    }

    tempEvent->temperature = mcp->readAmbient();
    return true;
  }

protected:
  Adafruit_MCP9601 *_MCP9601; ///< Pointer to MCP9601 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_MCP9601