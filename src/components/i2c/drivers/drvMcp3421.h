/*!
 * @file drvMcp3421.h
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
#ifndef DRV_MCP3421_H
#define DRV_MCP3421_H

#include "drvBase.h"
#include <Adafruit_MCP3421.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a MCP3421 sensor.
*/
/**************************************************************************/
class drvMcp3421 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for the MCP3421 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvMcp3421(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MCP3421 sensor.
  */
  /*******************************************************************************/
  ~drvMcp3421() { delete _mcp3421; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MCP3421 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _mcp3421 = new Adafruit_MCP3421();
    if (!_mcp3421->begin((uint8_t)_address, _i2c))
      return false;

    return configureSensor();
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

#endif // drvMcp3421