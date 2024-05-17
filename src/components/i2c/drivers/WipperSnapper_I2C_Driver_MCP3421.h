/*!
 * @file WipperSnapper_I2C_Driver_MCP3421.h
 *
 * Device driver for the MCP3421 18-bit ADC sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2024 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_MCP3421_H
#define WipperSnapper_I2C_Driver_MCP3421_H

#include "WipperSnapper_I2C_Driver.h"
#include "Wippersnapper.h"
#include <Adafruit_MCP3421.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a MCP3421 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MCP3421 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for the MCP3421 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MCP3421(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MCP3421 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MCP3421() { delete _mcp3421; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MCP3421 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _mcp3421 = new Adafruit_MCP3421();
    if (_mcp3421->begin((uint8_t)_sensorAddress, _i2c)) {
      WS_DEBUG_PRINTLN("Failed to find MCP3421 chip");
      return false;
    }

    // NOTE: We should allow the gain to be set in future, like resolution
    //  12_BIT (240 SPS), 14_BIT (60 SPS), 16_BIT (15 SPS), 18_BIT (3.75 SPS)
    _mcp3421->setResolution(RESOLUTION_18_BIT);
    if (_mcp3421->getResolution() != RESOLUTION_18_BIT) {
      WS_DEBUG_PRINTLN("Failed to set resolution");
      return false;
    }

    _mcp3421->setMode(MODE_ONE_SHOT);
    if (_mcp3421->getMode() != MODE_ONE_SHOT) {
      WS_DEBUG_PRINTLN("Failed to set mode");
      return false;
    }
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
    ulong start = millis();
    if (!_mcp3421->startOneShotConversion()) {
      WS_DEBUG_PRINTLN("Failed to start one-shot conversion");
      return false;
    }
    while (!_mcp3421->isReady()) {
      if (millis() - start > 1000) {
        WS_DEBUG_PRINTLN("Timeout waiting for conversion to complete");
        return false;
      }
    }
    voltageEvent->voltage = (float)_mcp3421->readADC();
    return true;
  }

protected:
  Adafruit_MCP3421 *_mcp3421; ///< Pointer to MCP3421 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_MCP3421