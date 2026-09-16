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
#define DS2484_RECOVER_AFTER                                                   \
  3 ///< Consecutive 1-Wire failures before the bridge
    ///< is reset and the bus re-searched
#define DS18B20_POWER_ON_RAW                                                   \
  0x0550 ///< Scratchpad value (85C) before any
         ///< conversion has run

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

    // check bus is okay, then locate the first DS18B20 on it
    if (!_ds2484->OneWireReset())
      return false;
    if (!findDs18b20())
      return false;
    return true;
  }

  /*!
      @brief    Searches the 1-Wire bus for the first DS18B20 and records its
                ROM address. Used at begin() and again during recovery, so a
                replaced sensor's new address is picked up.
      @returns  True if a DS18B20 was found, False otherwise (the previous
                ROM, if any, is kept).
  */
  bool findDs18b20() {
    if (!_ds2484->OneWireReset())
      return false;
    _ds2484->OneWireSearchReset();
    uint8_t rom[8];
    while (_ds2484->OneWireSearch(rom)) {
      if (rom[0] == DS18B20_FAMILY_CODE) {
        memcpy(_rom, rom, sizeof(_rom));
        return true;
      }
    }
    return false;
  }

  /*!
      @brief    Addresses the DS18B20 on the OneWire bus: reset, then Match ROM
                with the recorded address.
      @returns  True if the bus reset succeeded and a device is present, False
                otherwise.
  */
  bool selectDevice() {
    if (!_ds2484->OneWireReset() || !_ds2484->presencePulseDetected())
      return false;
    if (!_ds2484->OneWireWriteByte(DS18B20_CMD_MATCH_ROM))
      return false;
    for (size_t i = 0; i < sizeof(_rom); i++) {
      if (!_ds2484->OneWireWriteByte(_rom[i]))
        return false;
    }
    return true;
  }

  /*!
      @brief    Dallas/Maxim 1-Wire CRC8 (polynomial 0x31 reflected, 0x8C), as
                used for the DS18B20 scratchpad checksum.
      @param    data
                Bytes to checksum.
      @param    len
                Number of bytes.
      @returns  The CRC8 of the bytes.
  */
  static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    while (len--) {
      uint8_t inbyte = *data++;
      for (uint8_t i = 8; i; i--) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1;
        if (mix)
          crc ^= 0x8C;
        inbyte >>= 1;
      }
    }
    return crc;
  }

  /*!
      @brief    Records a failed 1-Wire step. The step is simply retried on
                the next tick; after DS2484_RECOVER_AFTER consecutive failures
                the DS2484 bridge is reset (clearing a stuck or shorted bus
                state) and the bus re-searched, so a sensor that was removed,
                rewired or replaced is picked up again. Recovery runs at most
                once per lead window.
  */
  void noteFailure() {
    if (++_fail_count < DS2484_RECOVER_AFTER || _recovered)
      return;
    _fail_count = 0;
    _recovered = true;
    WS_DEBUG_PRINTLN("DS2484: repeated 1-Wire failures, resetting bridge and "
                     "re-searching for a DS18B20");
    _ds2484->reset();
    if (!findDs18b20())
      WS_DEBUG_PRINTLN("DS2484: no DS18B20 found on the bus");
  }

  /*!
      @brief    Background conversion step, called every DS2484_TICK_MS while
                a read is pending. The first tick of a window starts a
                temperature conversion; once DS18B20_CONVERT_MS has elapsed
                the scratchpad is read, validated (CRC, and not the 85C
                power-on value a missing or wrong-ROM device yields) and filed
                with NewSample(). Any step that fails is retried from the top
                on the next tick, with bus recovery after repeated failures;
                no sample is filed until a valid conversion is read, so a
                missing sensor stops publishing rather than publishing NAN.
  */
  void fastTick() override {
    ulong now = millis();
    if (_first_tick) {
      _converting = false; // any conversion from a previous window is stale
      _recovered = false;
    }

    if (!_converting) {
      if (!selectDevice() ||
          !_ds2484->OneWireWriteByte(DS18B20_CMD_CONVERT_T)) {
        noteFailure();
        return; // retried next tick
      }
      _converting = true;
      _convert_start = now;
      return;
    }

    if (now - _convert_start < DS18B20_CONVERT_MS)
      return; // still converting

    _converting = false; // whatever happens next, the next tick starts afresh
    if (!selectDevice() ||
        !_ds2484->OneWireWriteByte(DS18B20_CMD_READ_SCRATCHPAD)) {
      noteFailure();
      return;
    }
    uint8_t data[9];
    for (size_t i = 0; i < sizeof(data); i++) {
      if (!_ds2484->OneWireReadByte(&data[i])) {
        noteFailure();
        return;
      }
    }
    int16_t raw = (data[1] << 8) | data[0];
    // Reject garbage: a CRC mismatch (no device, or a replaced one that does
    // not answer to the old ROM, reads as all 1s) or the power-on value,
    // which means the conversion never ran.
    if (crc8(data, 8) != data[8] || raw == DS18B20_POWER_ON_RAW) {
      noteFailure();
      return;
    }
    _fail_count = 0;
    _temperature = (float)raw / 16.0f;
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
  uint8_t _fail_count = 0;  ///< Consecutive failed 1-Wire steps
  bool _recovered = false;  ///< Bus recovery already ran this lead window
};

#endif // drvDs2484