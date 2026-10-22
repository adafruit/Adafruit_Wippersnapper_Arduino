/*!
 * @file drvAs7343.h
 *
 * Device driver for the AS7343 14-channel multi-spectral sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_AS7343_H
#define DRV_AS7343_H

#include "drvBase.h"
#include <Adafruit_AS7343.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for an AS7343 sensor.
            Every spectral channel is a "light" subcomponent (raw ADC counts,
            not lux), so the device carries many TypesEntry of the same
            sensor type; GetSensorEvent() switches on the broker-assigned
            entry.key to pick the channel.
*/
/**************************************************************************/
class drvAs7343 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an AS7343 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvAs7343(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AS7343 sensor.
  */
  /*******************************************************************************/
  ~drvAs7343() { delete _as7343; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AS7343 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_as7343)
      delete _as7343;
    _as7343 = new Adafruit_AS7343();
    return _as7343->begin((uint8_t)_address, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the AS7343 sensor with default settings. The
                18-channel auto-SMUX mode is pinned because the channel table
                in GetSensorEvent() indexes the 18-value readAllChannels()
                buffer.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    bool ok = true;
    // Explicitly set defaults to pin behavior against library changes.
    ok &= _as7343->setGain(AS7343_GAIN_64X);
    _atime = 29; // 30 x 1.67ms steps = ~50ms
    ok &= _as7343->setATIME(_atime);
    ok &= _as7343->setASTEP(kAstep);
    ok &= _as7343->setSMUXMode(AS7343_SMUX_18CH);
    // TODO: expose the onboard LED (enableLED/setLEDCurrent) as a setting
    ok &= _as7343->enableLED(false);
    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the gain setting to the driver.
      @param    gain
                The gain index from the broker, which equals as7343_gain_t
                (0=0.5x, 1=1x, 2=2x, 3=4x, 4=8x, 5=16x, 6=32x, 7=64x,
                8=128x, 9=256x, 10=512x, 11=1024x, 12=2048x).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setGain(const ws_config_Value &gain) override {
    if (gain.which_value != ws_config_Value_int_value_tag)
      return false;
    if (gain.value.int_value < AS7343_GAIN_0_5X ||
        gain.value.int_value > AS7343_GAIN_2048X)
      return false;
    return _as7343->setGain((as7343_gain_t)gain.value.int_value);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the integration time setting to the driver. ASTEP is
                fixed (1.67ms per step) and ATIME picks the step count. One
                read runs 3 integrations (18-channel auto-SMUX) and the
                library gives up after 1s, so the longest option is ~200ms.
      @param    integration_time
                The integration time index from the broker
                (0=5ms, 1=10ms, 2=25ms, 3=50ms, 4=100ms, 5=200ms).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setIntegrationTime(const ws_config_Value &integration_time) override {
    if (integration_time.which_value != ws_config_Value_int_value_tag)
      return false;
    uint8_t atime;
    switch (integration_time.value.int_value) {
    case 0:
      atime = 2; // 5ms
      break;
    case 1:
      atime = 5; // 10ms
      break;
    case 2:
      atime = 14; // 25ms
      break;
    case 3:
      atime = 29; // 50ms
      break;
    case 4:
      atime = 59; // 100ms
      break;
    case 5:
      atime = 119; // 200ms
      break;
    default:
      return false;
    }
    if (!_as7343->setATIME(atime))
      return false;
    _atime = atime;
    return true;
  }

  /*!
      @brief    Runs one 18-channel auto-SMUX measurement into the cache. The
                library starts the measurement and blocks until all three
                SMUX cycles complete (~3x the integration time). Rejected
                when the analog front end saturated: the gain is too high for
                the light level.
      @returns  True if the cache holds a valid sample, False otherwise.
  */
  bool ReadSensorData() override {
    uint16_t readings[18];
    if (!_as7343->readAllChannels(readings) || _as7343->isAnalogSaturated())
      return false;
    memcpy(_readings, readings, sizeof(_readings));
    return true;
  }

  /*!
      @brief    Reads one spectral channel, selected by the broker-assigned
                entry.key (the subcomponent's index in the component
                definition), as a light event holding the channel's raw ADC
                counts. Falls back to the base handler for anything that is
                not a known light channel.
      @param    entry
                The TypesEntry to read.
      @param    sensors_event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise (including a digitally saturated channel).
  */
  bool GetSensorEvent(const ws_i2c_Add_TypesEntry &entry,
                      sensors_event_t *sensors_event) override {
    // Spectral channel for each entry.key, in the same order as the
    // subcomponents in Wippersnapper_Components i2c/as7343/definition.json -
    // keep the two in sync.
    // TODO: the flicker (FD) channel and the duplicate VIS readings from SMUX
    // cycles 2 and 3 are not exposed.
    static const as7343_channel_t kKeyChannels[] = {
        AS7343_CHANNEL_F1,       AS7343_CHANNEL_F2,  AS7343_CHANNEL_FZ,
        AS7343_CHANNEL_F3,       AS7343_CHANNEL_F4,  AS7343_CHANNEL_F5,
        AS7343_CHANNEL_FY,       AS7343_CHANNEL_FXL, AS7343_CHANNEL_F6,
        AS7343_CHANNEL_F7,       AS7343_CHANNEL_F8,  AS7343_CHANNEL_NIR,
        AS7343_CHANNEL_VIS_TL_0,
    };
    if (entry.value != ws_sensor_Type_T_LIGHT ||
        entry.key >= sizeof(kKeyChannels) / sizeof(kKeyChannels[0]))
      return drvBase::GetSensorEvent(entry, sensors_event);
    if (!AttemptRead())
      return false;
    uint16_t counts = _readings[kKeyChannels[entry.key]];
    // ADC full scale is (ATIME+1) * (ASTEP+1) counts, capped at 16 bits
    uint32_t full_scale = (uint32_t)(_atime + 1) * (kAstep + 1);
    if (full_scale > UINT16_MAX)
      full_scale = UINT16_MAX;
    if (counts >= full_scale)
      return false;
    sensors_event->light = counts;
    return true;
  }

protected:
  Adafruit_AS7343 *_as7343 = nullptr;     ///< Pointer to AS7343 sensor object
  uint16_t _readings[18] = {0};           ///< Last 18-channel auto-SMUX sample
  uint8_t _atime = 29;                    ///< Current ATIME (integration steps)
  static constexpr uint16_t kAstep = 599; ///< ASTEP: 600 x 2.78us = 1.67ms
};

#endif // DRV_AS7343_H
