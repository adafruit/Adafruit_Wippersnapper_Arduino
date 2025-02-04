/*!
 * @file drvDs2484.h
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

#ifndef DRV_DS2484_H
#define DRV_DS2484_H

#define DS18B20_FAMILY_CODE 0x28         ///< DS18B20 family code
#define DS18B20_CMD_CONVERT_T 0x44       ///< Convert T command
#define DS18B20_CMD_MATCH_ROM 0x55       ///< Match ROM command
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE ///< Read Scratchpad command

#include "drvBase.h"
#include <Adafruit_DS248x.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the DS2484 I2C OneWire
            converter hosting a DS18b20 temperature sensor.
*/
/**************************************************************************/
class drvDs2484 : public drvBase {

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
  drvDs2484(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
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
      @brief    Destructor for an DS2484 sensor.
  */
  /*******************************************************************************/
  ~drvDs2484() { delete _ds2484; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the DS2484 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    // initialize DS2484
    _ds2484 = new Adafruit_DS248x();
    if (!_ds2484->begin(_i2c, (uint8_t)_address))
      return false;

    // check bus is okay
    if (!_ds2484->OneWireReset())
      return false;

    // locate first DS18B20
    bool found_device = false;
    _ds2484->OneWireReset();
    _ds2484->OneWireSearchReset();
    while (!found_device && _ds2484->OneWireSearch(_rom)) {
      if (_rom[0] == DS18B20_FAMILY_CODE) {
        found_device = true;
      }
    }

    if (!found_device)
      return false;

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
    if (!_ds2484->OneWireReset())
      return false;

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
    if (!_ds2484->OneWireReset()) {
      return false;
    }
    _ds2484->OneWireWriteByte(DS18B20_CMD_MATCH_ROM); // Match ROM command
    for (int i = 0; i < 8; i++) {
      _ds2484->OneWireWriteByte(_rom[i]);
    }
    _ds2484->OneWireWriteByte(
        DS18B20_CMD_READ_SCRATCHPAD); // Read Scratchpad command

    uint8_t data[9];
    for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++) {
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

#endif // drvDs2484