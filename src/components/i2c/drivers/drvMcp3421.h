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

#define MCP3421_TICK_MS 50        ///< Poll the conversion every 50ms
#define MCP3421_READ_LEAD_MS 1000 ///< Start converting 1s before a read is due
#define MCP3421_CONVERT_TIMEOUT_MS 500 ///< Restart a conversion stuck this long

/*!
    @brief  Class that provides a driver interface for a MCP3421 sensor.
*/
class drvMcp3421 : public drvBase {
public:
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
  drvMcp3421(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // An 18-bit one-shot conversion takes ~270ms, so it is started and
    // collected by fastTick() during the lead window before each read is due
    // rather than blocking the read pass.
    _fast_tick_ms = MCP3421_TICK_MS;
    _tick_lead_ms = MCP3421_READ_LEAD_MS;
  }

  /*!
      @brief    Destructor for an MCP3421 sensor.
  */
  ~drvMcp3421() { delete _mcp3421; }

  /*!
      @brief    Initializes the MCP3421 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _mcp3421 = new Adafruit_MCP3421();
    if (!_mcp3421->begin((uint8_t)_address, _i2c))
      return false;

    return configureSensor();
  }

  /*!
      @brief    Configures the MCP3421 sensor.
      @returns  True if the sensor was configured successfully, False otherwise.
  */
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

  /*!
      @brief    Background conversion step, called every MCP3421_TICK_MS while
                a read is pending. Starts a one-shot conversion, then reads the
                ADC once isReady() and files the sample with NewSample(). A
                conversion that does not complete within
                MCP3421_CONVERT_TIMEOUT_MS is restarted.
  */
  void fastTick() override {
    ulong now = millis();
    if (_first_tick)
      _converting = false; // any conversion from a previous window is stale

    if (!_converting || now - _convert_start > MCP3421_CONVERT_TIMEOUT_MS) {
      if (_converting)
        WS_DEBUG_PRINTLN("MCP3421: conversion timed out, restarting");
      if (!_mcp3421->startOneShotConversion()) {
        WS_DEBUG_PRINTLN("Failed to start one-shot conversion");
        return;
      }
      _converting = true;
      _convert_start = now;
      return;
    }

    if (!_mcp3421->isReady())
      return;
    _converting = false;
    _adc = _mcp3421->readADC();
    NewSample();
  }

  /*!
      @brief    Gets the ADC reading, from the most recent conversion collected
                by fastTick().
      @param    rawEvent
                ADC sensor reading
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!AttemptRead())
      return false;
    rawEvent->data[0] = (float)_adc;
    return true;
  }

protected:
  Adafruit_MCP3421 *_mcp3421; ///< Pointer to MCP3421 sensor object
  int32_t _adc = 0;           ///< Last ADC conversion result
  bool _converting = false;   ///< A one-shot conversion is in flight
  ulong _convert_start = 0;   ///< millis() the in-flight conversion started
};

#endif // drvMcp3421