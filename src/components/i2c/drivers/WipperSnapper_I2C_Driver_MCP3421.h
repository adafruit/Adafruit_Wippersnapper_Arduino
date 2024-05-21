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
    if (!_mcp3421->begin((uint8_t)_sensorAddress, _i2c)) {
      WS_DEBUG_PRINTLN("Failed to find MCP3421 chip");
      return false;
    }

    if (!configureSensor()) {
      WS_DEBUG_PRINTLN("Failed to configure MCP3421 sensor");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the MCP3421 sensor.
      @returns  True if the sensor was configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureSensor() {
    // NOTE: We should allow the gain to be set in future, like resolution
    //  12_BIT (240 SPS), 14_BIT (60 SPS), 16_BIT (15 SPS), 18_BIT (3.75 SPS)
    _mcp3421->setResolution(RESOLUTION_18_BIT);
    if (_mcp3421->getResolution() != RESOLUTION_18_BIT) {
      WS_DEBUG_PRINTLN("Failed to set resolution to 18-bit");
      return false;
    }

    _mcp3421->setGain(GAIN_8X);
    if (_mcp3421->getGain() != GAIN_8X) {
      WS_DEBUG_PRINTLN("Failed to set gain to 8x");
      return false;
    }

    _mcp3421->setMode(MODE_ONE_SHOT);
    if (_mcp3421->getMode() != MODE_ONE_SHOT) {
      WS_DEBUG_PRINTLN("Failed to set mode to One-Shot");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the ADC sensor with short wait for data.
      @param    rawEvent
                ADC sensor reading
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    ulong start = millis();
    if (!_mcp3421->startOneShotConversion()) {
      WS_DEBUG_PRINTLN("Failed to start one-shot conversion");
      return false;
    }
    while (!_mcp3421->isReady()) {
      ulong newMillis = millis();
      if (newMillis - start > 500) {
        WS_DEBUG_PRINTLN("Timeout waiting for conversion to complete");
        return false;
      } else if (start > newMillis) {
        start = millis(); // rollover
      }
      delay(50);
    }
    rawEvent->data[0] = (float)_mcp3421->readADC();
    return true;
  }

protected:
  Adafruit_MCP3421 *_mcp3421; ///< Pointer to MCP3421 sensor object
};

#endif // WipperSnapper_I2C_Driver_MCP3421