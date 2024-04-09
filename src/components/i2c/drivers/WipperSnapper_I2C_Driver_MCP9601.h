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
class WipperSnapper_I2C_Driver_MCP9601 : public WipperSnapper_I2C_Driver
{
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MCP9601 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MCP9601(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress)
  {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MCP9601 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MCP9601()
  {
    // Called when a MCP9601 component is deleted.
    delete _MCP9601;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MCP9601 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin()
  {
    _MCP9601 = new Adafruit_MCP9601();
    if (!_MCP9601->begin((uint8_t)_sensorAddress, _i2c))
    {
      return false;
    }
    _MCP9601->setADCresolution(MCP9600_ADCRESOLUTION_18);
    _MCP9601->setThermocoupleType(MCP9600_TYPE_K);
    _MCP9601->setFilterCoefficient(3);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MCP9601's current ambient temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent)
  {
    tempEvent->temperature = _MCP9601->readAmbient();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MCP9601's current thermocouple temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent)
  {
    uint8_t status = _MCP9601->getStatus();
    if (status & MCP9601_STATUS_OPENCIRCUIT)
    {
      WS_DEBUG_PRINTLN("Thermocouple open!");
      return false;
    }
    if (status & MCP9601_STATUS_SHORTCIRCUIT)
    {
      WS_DEBUG_PRINTLN("Thermocouple shorted to ground!");
      return false;
    }
    rawEvent->data[0] = _MCP9601->readThermocouple();
    return true;
  }

protected:
  Adafruit_MCP9601 *_MCP9601; ///< Pointer to MCP9601 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_MCP9601