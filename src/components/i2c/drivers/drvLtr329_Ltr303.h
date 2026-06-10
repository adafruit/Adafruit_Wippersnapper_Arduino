/*!
 * @file drvLtr329_Ltr303.h
 *
 * Device driver for the LTR329 + LTR303 (329+interrupt) light sensors.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_LTR329_LTR303_H
#define DRV_LTR329_LTR303_H

#include "drvBase.h"
#include <Adafruit_LTR329_LTR303.h>

/*!
    @brief  Class that provides a driver interface for a LTR329/303 sensor.
*/
class drvLtr329_Ltr303 : public drvBase {
public:
  /*!
      @brief    Constructor for a LTR329/303 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvLtr329_Ltr303(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                   const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an LTR329/303 sensor.
  */
  ~drvLtr329_Ltr303() { delete _LTR329; }

  /*!
      @brief    Initializes the LTR329/303 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _LTR329 = new Adafruit_LTR329();
    // Attempt to initialize LTR329
    if (!_LTR329->begin(_i2c))
      return false;

    return true;
  }

  /*!
      @brief    Configures the LTR329/303 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  bool configureDefaults() override {
    // Configure LTR329 sensor - tested on a dull British day Oct'23 @7kLux
    // Matches similar lux value from LTR390 with default configuration.
    _LTR329->setGain(LTR3XX_GAIN_48);
    _LTR329->setIntegrationTime(LTR3XX_INTEGTIME_100);
    _LTR329->setMeasurementRate(LTR3XX_MEASRATE_100);
    return true;
  }

  /*!
      @brief    Applies the ALS gain setting to the driver.
      @param    gain
                The gain index from the broker
                (0=1x, 1=2x, 2=4x, 3=8x, 4=48x, 5=96x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setGain(const ws_config_Value &gain) override {
    if (gain.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = gain.value.int_value;
    ltr329_gain_t als_gain;
    switch (val) {
    case 0:
      als_gain = LTR3XX_GAIN_1;
      break;
    case 1:
      als_gain = LTR3XX_GAIN_2;
      break;
    case 2:
      als_gain = LTR3XX_GAIN_4;
      break;
    case 3:
      als_gain = LTR3XX_GAIN_8;
      break;
    case 4:
      als_gain = LTR3XX_GAIN_48;
      break;
    case 5:
      als_gain = LTR3XX_GAIN_96;
      break;
    default:
      als_gain = LTR3XX_GAIN_48;
      break;
    }
    _LTR329->setGain(als_gain);
    return true;
  }

  /*!
      @brief    Applies the ALS integration time setting to the driver.
      @param    integration_time
                The integration time index from the broker
                (0=50ms, 1=100ms, 2=150ms, 3=200ms, 4=250ms, 5=300ms,
                6=350ms, 7=400ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setIntegrationTime(const ws_config_Value &integration_time) override {
    if (integration_time.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = integration_time.value.int_value;
    ltr329_integrationtime_t int_time;
    switch (val) {
    case 0:
      int_time = LTR3XX_INTEGTIME_50;
      break;
    case 1:
      int_time = LTR3XX_INTEGTIME_100;
      break;
    case 2:
      int_time = LTR3XX_INTEGTIME_150;
      break;
    case 3:
      int_time = LTR3XX_INTEGTIME_200;
      break;
    case 4:
      int_time = LTR3XX_INTEGTIME_250;
      break;
    case 5:
      int_time = LTR3XX_INTEGTIME_300;
      break;
    case 6:
      int_time = LTR3XX_INTEGTIME_350;
      break;
    case 7:
      int_time = LTR3XX_INTEGTIME_400;
      break;
    default:
      int_time = LTR3XX_INTEGTIME_100;
      break;
    }
    _LTR329->setIntegrationTime(int_time);
    return true;
  }

  /*!
      @brief    Applies the ALS measurement rate setting to the driver.
      @param    measurement_rate
                The measurement rate index from the broker
                (0=50ms, 1=100ms, 2=200ms, 3=500ms, 4=1000ms, 5=2000ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setMeasurementRate(const ws_config_Value &measurement_rate) override {
    if (measurement_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = measurement_rate.value.int_value;
    ltr329_measurerate_t meas_rate;
    switch (val) {
    case 0:
      meas_rate = LTR3XX_MEASRATE_50;
      break;
    case 1:
      meas_rate = LTR3XX_MEASRATE_100;
      break;
    case 2:
      meas_rate = LTR3XX_MEASRATE_200;
      break;
    case 3:
      meas_rate = LTR3XX_MEASRATE_500;
      break;
    case 4:
      meas_rate = LTR3XX_MEASRATE_1000;
      break;
    case 5:
      meas_rate = LTR3XX_MEASRATE_2000;
      break;
    default:
      meas_rate = LTR3XX_MEASRATE_100;
      break;
    }
    _LTR329->setMeasurementRate(meas_rate);
    return true;
  }

  /*!
      @brief    Reads both LTR329 channels into the cache when a new conversion
                is available. The LTR329 produces visible+IR and IR-only in a
                single conversion and reading clears the data-ready flag, so
                both the Light and Raw events are served from one read.
      @returns  True if a valid reading is cached, False otherwise.
  */
  bool readBothChannels() {
    if (_LTR329->newDataAvailable()) {
      _LTR329->readBothChannels(_visible_plus_ir, _infrared);
      _has_reading = true;
    }
    return _has_reading;
  }

  /*!
      @brief    Reads the LTR329's ambient light level ([Visible+IR] - IR-only)
      @param    lightEvent
                Light sensor reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!readBothChannels())
      return false;
    lightEvent->light = _visible_plus_ir - _infrared;
    return true;
  }

  /*!
      @brief    Reads the LTR329's infrared value into an event.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!readBothChannels())
      return false;
    rawEvent->data[0] = (float)_infrared;
    return true;
  }

protected:
  Adafruit_LTR329 *_LTR329;      ///< Pointer to LTR329 light sensor object
  uint16_t _visible_plus_ir = 0; ///< Cached visible+IR channel reading
  uint16_t _infrared = 0;        ///< Cached IR-only channel reading
  bool _has_reading = false;     ///< True once a conversion has been cached
};

#endif // drvLtr329_Ltr303