/*!
 * @file drvAs7331.h
 *
 * Device driver for the AS7331 UV spectral sensor (UVA/UVB/UVC).
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
#ifndef DRV_AS7331_H
#define DRV_AS7331_H

#include "drvBase.h"
#include <Adafruit_AS7331.h>

#define AS7331_BREAKTIME_200US 25 ///< Recommended break time (val * 8µs)

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for an AS7331 UV sensor.
*/
/**************************************************************************/
class drvAs7331 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an AS7331 sensor.
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
  drvAs7331(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AS7331 sensor.
  */
  /*******************************************************************************/
  ~drvAs7331() { delete _as7331; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AS7331 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_as7331)
      delete _as7331;
    _as7331 = new Adafruit_AS7331();
    if (!_as7331->begin(_i2c, (uint8_t)_address))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the AS7331 sensor with default settings. The device
                must be powered down before changing settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    bool ok = true;
    // Configure sensor — must power down before changing settings
    ok &= _as7331->powerDown(true);
    if (!ok)
      return false;
    // Explicitly set defaults to pin behavior against library changes.
    ok &= _as7331->setGain(AS7331_GAIN_4X);
    ok &= _as7331->setIntegrationTime(AS7331_TIME_64MS);
    ok &= _as7331->setMeasurementMode(AS7331_MODE_CONT);
    ok &= _as7331->setClockFrequency(AS7331_CLOCK_1024MHZ);
    ok &= _as7331->setBreakTime(AS7331_BREAKTIME_200US);
    ok &= _as7331->setStandby(false);

    // Start continuous measurements
    ok &= _as7331->powerDown(false);

    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the gain setting to the driver. Higher gain trades
                saturation range for sensitivity. The device is bracketed
                with a power-down/up around the change.
      @param    gain
                The gain index from the broker
                (0=2048x, 1=1024x, 2=512x, 3=256x, 4=128x, 5=64x, 6=32x,
                7=16x, 8=8x, 9=4x, 10=2x, 11=1x).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setGain(const ws_config_Value &gain) override {
    if (gain.which_value != ws_config_Value_int_value_tag)
      return false;
    as7331_gain_t g;
    switch (gain.value.int_value) {
    case 0:
      g = AS7331_GAIN_2048X;
      break;
    case 1:
      g = AS7331_GAIN_1024X;
      break;
    case 2:
      g = AS7331_GAIN_512X;
      break;
    case 3:
      g = AS7331_GAIN_256X;
      break;
    case 4:
      g = AS7331_GAIN_128X;
      break;
    case 5:
      g = AS7331_GAIN_64X;
      break;
    case 6:
      g = AS7331_GAIN_32X;
      break;
    case 7:
      g = AS7331_GAIN_16X;
      break;
    case 8:
      g = AS7331_GAIN_8X;
      break;
    case 9:
      g = AS7331_GAIN_4X;
      break;
    case 10:
      g = AS7331_GAIN_2X;
      break;
    case 11:
      g = AS7331_GAIN_1X;
      break;
    default:
      return false;
    }
    _as7331->powerDown(true);
    bool ok = _as7331->setGain(g);
    _as7331->powerDown(false);
    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the integration time setting to the driver. The device
                is bracketed with a power-down/up around the change.
      @param    integration_time
                The integration time index from the broker
                (0=1ms, 1=2ms, 2=4ms, 3=8ms, 4=16ms, 5=32ms, 6=64ms, 7=128ms,
                8=256ms, 9=512ms, 10=1024ms, 11=2048ms, 12=4096ms, 13=8192ms,
                14=16384ms).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setIntegrationTime(const ws_config_Value &integration_time) override {
    if (integration_time.which_value != ws_config_Value_int_value_tag)
      return false;
    as7331_time_t t;
    switch (integration_time.value.int_value) {
    case 0:
      t = AS7331_TIME_1MS;
      break;
    case 1:
      t = AS7331_TIME_2MS;
      break;
    case 2:
      t = AS7331_TIME_4MS;
      break;
    case 3:
      t = AS7331_TIME_8MS;
      break;
    case 4:
      t = AS7331_TIME_16MS;
      break;
    case 5:
      t = AS7331_TIME_32MS;
      break;
    case 6:
      t = AS7331_TIME_64MS;
      break;
    case 7:
      t = AS7331_TIME_128MS;
      break;
    case 8:
      t = AS7331_TIME_256MS;
      break;
    case 9:
      t = AS7331_TIME_512MS;
      break;
    case 10:
      t = AS7331_TIME_1024MS;
      break;
    case 11:
      t = AS7331_TIME_2048MS;
      break;
    case 12:
      t = AS7331_TIME_4096MS;
      break;
    case 13:
      t = AS7331_TIME_8192MS;
      break;
    case 14:
      t = AS7331_TIME_16384MS;
      break;
    default:
      return false;
    }
    _as7331->powerDown(true);
    bool ok = _as7331->setIntegrationTime(t);
    _as7331->powerDown(false);
    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the measurement mode setting to the driver. The device
                is bracketed with a power-down/up around the change.
      @param    mode
                The measurement mode index from the broker
                (0=continuous, 1=command, 2=synchronized start,
                3=synchronized data).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMode(const ws_config_Value &mode) override {
    if (mode.which_value != ws_config_Value_int_value_tag)
      return false;
    as7331_mode_t m;
    switch (mode.value.int_value) {
    case 0:
      m = AS7331_MODE_CONT;
      break;
    case 1:
      m = AS7331_MODE_CMD;
      break;
    case 2:
      m = AS7331_MODE_SYNS;
      break;
    case 3:
      m = AS7331_MODE_SYND;
      break;
    default:
      return false;
    }
    _as7331->powerDown(true);
    bool ok = _as7331->setMeasurementMode(m);
    _as7331->powerDown(false);
    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the clock frequency setting to the driver. The device
                is bracketed with a power-down/up around the change.
      @param    clock_frequency
                The clock frequency index from the broker
                (0=1.024MHz, 1=2.048MHz, 2=4.096MHz, 3=8.192MHz).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setClockFrequency(const ws_config_Value &clock_frequency) override {
    if (clock_frequency.which_value != ws_config_Value_int_value_tag)
      return false;
    as7331_clock_t c;
    switch (clock_frequency.value.int_value) {
    case 0:
      c = AS7331_CLOCK_1024MHZ;
      break;
    case 1:
      c = AS7331_CLOCK_2048MHZ;
      break;
    case 2:
      c = AS7331_CLOCK_4096MHZ;
      break;
    case 3:
      c = AS7331_CLOCK_8192MHZ;
      break;
    default:
      return false;
    }
    _as7331->powerDown(true);
    bool ok = _as7331->setClockFrequency(c);
    _as7331->powerDown(false);
    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AS7331's current UVB reading in uW/cm2 as raw event.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    float uva, uvb, uvc;
    if (!_as7331->readAllUV_uWcm2(&uva, &uvb, &uvc))
      return false;

    WS_DEBUG_PRINT("AS7331 UVA: ");
    WS_DEBUG_PRINTVAR(uva);
    WS_DEBUG_PRINT(" UVB: ");
    WS_DEBUG_PRINTVAR(uvb);
    WS_DEBUG_PRINT(" UVC: ");
    WS_DEBUG_PRINTLNVAR(uvc);

    rawEvent->data[0] = uvb;
    return true;
  }

protected:
  Adafruit_AS7331 *_as7331 = nullptr; ///< Pointer to AS7331 sensor object
};

#endif // DRV_AS7331_H
