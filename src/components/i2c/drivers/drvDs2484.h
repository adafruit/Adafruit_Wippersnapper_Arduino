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
#define DS18B20_CONVERT_MS 750   ///< 12-bit conversion time (datasheet max)
#define DS2484_TICK_MS 50        ///< Poll the conversion every 50ms
#define DS2484_READ_LEAD_MS 1000 ///< Start converting 1s before a read is due

#include "drvBase.h"
#include <Adafruit_DS248x.h>

/*!
    @brief  Class that provides a sensor driver for the DS2484 I2C OneWire
            converter hosting a DS18b20 temperature sensor.
*/
class drvDs2484 : public drvBase {

public:
  /*!
      @brief    Constructor for a DS2484 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvDs2484(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // A DS18B20 conversion takes 750ms, so it is started and collected by
    // fastTick() during the lead window before each read is due rather than
    // blocking the read pass.
    _fast_tick_ms = DS2484_TICK_MS;
    _tick_lead_ms = DS2484_READ_LEAD_MS;
  }

  /*!
      @brief    Destructor for an DS2484 sensor.
  */
  ~drvDs2484() { delete _ds2484; }

  /*!
      @brief    Initializes the DS2484 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
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

  /*!
      @brief    Addresses the DS18B20 on the OneWire bus: reset, then Match ROM
                with the address found in begin().
      @returns  True if the bus reset succeeded and a device is present, False
                otherwise.
  */
  bool selectDevice() {
    if (!_ds2484->OneWireReset() || !_ds2484->presencePulseDetected())
      return false;
    _ds2484->OneWireWriteByte(DS18B20_CMD_MATCH_ROM);
    for (int i = 0; i < 8; i++) {
      _ds2484->OneWireWriteByte(_rom[i]);
    }
    return true;
  }

  /*!
      @brief    Background conversion step, called every DS2484_TICK_MS while
                a read is pending. The first tick of a window starts a
                temperature conversion; once DS18B20_CONVERT_MS has elapsed
                the scratchpad is read and the sample filed with NewSample().
                A DS18B20 that has disappeared from the bus files NAN, as the
                blocking version did.
  */
  void fastTick() override {
    ulong now = millis();
    if (_first_tick)
      _converting = false; // any conversion from a previous window is stale

    if (!_converting) {
      if (!selectDevice()) {
        _temperature = NAN;
        NewSample();
        return;
      }
      _ds2484->OneWireWriteByte(DS18B20_CMD_CONVERT_T);
      _converting = true;
      _convert_start = now;
      return;
    }

    if (now - _convert_start < DS18B20_CONVERT_MS)
      return; // still converting

    _converting = false;
    if (!selectDevice())
      return; // lost the device mid-conversion; retry next tick
    _ds2484->OneWireWriteByte(DS18B20_CMD_READ_SCRATCHPAD);
    uint8_t data[9];
    for (size_t i = 0; i < sizeof(data); i++) {
      _ds2484->OneWireReadByte(&data[i]);
    }
    int16_t raw = (data[1] << 8) | data[0];
    _temperature = (float)raw / 16.0;
    NewSample();
  }

  /*!
      @brief    Gets the DS18B20's current temperature, from the most recent
                conversion collected by fastTick().
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!AttemptRead())
      return false;
    tempEvent->temperature = _temperature;
    return true;
  }

protected:
  Adafruit_DS248x *_ds2484; ///< DS2484 driver object
  uint8_t _rom[8];          ///< DS18B20 ROM
  float _temperature = NAN; ///< Last converted temperature, in C
  bool _converting = false; ///< A temperature conversion is in flight
  ulong _convert_start = 0; ///< millis() the in-flight conversion started
};

#endif // drvDs2484