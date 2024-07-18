/*!
 * @file WipperSnapper_I2C_Driver_DS2484.h
 *
 * Device driver the DS2484 I2C OneWire converter (hosting a DS18b20).
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

#ifndef WipperSnapper_I2C_Driver_DS2484_H
#define WipperSnapper_I2C_Driver_DS2484_H

#define DS18B20_FAMILY_CODE 0x28
#define DS18B20_CMD_CONVERT_T 0x44
#define DS18B20_CMD_MATCH_ROM 0x55
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_DS248x.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the DS2484 I2C OneWire
            converter hosting a DS18b20 temperature sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_DS2484 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a DS2484 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_DS2484(TwoWire *i2c,
                                  uint16_t sensorAddress = _DS2484_ADDRESS)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an DS2484 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_DS2484() { delete _ds2484; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the DS2484 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    // initialize DS2484
    _ds2484 = new Adafruit_DS248x();
    if (!_ds2484->begin(_i2c, (uint8_t)_sensorAddress)) {
      WS_DEBUG_PRINTLN("Could not find DS2484");
      return false;
    }

    // check bus is okay
    if (!_ds2484->OneWireReset()) {
      WS_DEBUG_PRINTLN("Failed to do a OneWire bus reset");
      if (_ds2484->shortDetected()) {
        WS_DEBUG_PRINTLN("\tShort detected");
      }
      if (!_ds2484->presencePulseDetected()) {
        WS_DEBUG_PRINTLN("\tNo presense pulse");
      }
      return false;
    }

    // locate first DS18B20
    bool found_device = false;
    _ds2484->OneWireReset();
    while (!found_device && _ds2484->OneWireSearch(_rom)) {
      if (_rom[0] == DS18B20_FAMILY_CODE) {
        found_device = true;
      }
    }

    if (!found_device) {
      WS_DEBUG_PRINTLN("Could not find DS18B20 attached to DS2484");
      return false;
    }

    WS_DEBUG_PRINTLN("DS2484 found");
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Processes a temperature event.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool processTemperatureEvent(sensors_event_t *tempEvent) {
    if (!_ds2484->OneWireReset()) {
      return false;
    }
    if (!_ds2484->presencePulseDetected()) {
      tempEvent->temperature = NAN;
      return true;
    }

    _ds2484->OneWireWriteByte(DS18B20_CMD_MATCH_ROM); // Match ROM command
    for (int i = 0; i < 8; i++) {
      _ds2484->OneWireWriteByte(_rom[i]);
    }

    // Start temperature conversion
    _ds2484->OneWireWriteByte(DS18B20_CMD_CONVERT_T); // Convert T command
    delay(750); // Wait for conversion (750ms for maximum precision)

    // Read scratchpad
    _ds2484->OneWireReset();
    _ds2484->OneWireWriteByte(DS18B20_CMD_MATCH_ROM); // Match ROM command
    for (int i = 0; i < 8; i++) {
      _ds2484->OneWireWriteByte(_rom[i]);
    }
    _ds2484->OneWireWriteByte(
        DS18B20_CMD_READ_SCRATCHPAD); // Read Scratchpad command

    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
      _ds2484->OneWireReadByte(&data[i]);
    }

    // Calculate temperature
    int16_t raw = (data[1] << 8) | data[0];
    tempEvent->temperature = (float)raw / 16.0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the DS2484's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return processTemperatureEvent(tempEvent);
  }

protected:
  Adafruit_DS248x *_ds2484; ///< DS2484 driver object
  uint8_t _rom[8];          ///< DS18B20 ROM
};

#endif // WipperSnapper_I2C_Driver_DS2484