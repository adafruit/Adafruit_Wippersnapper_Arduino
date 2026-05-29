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
  bool setGain(int32_t gain) override {
    ltr329_gain_t als_gain;
    switch (gain) {
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
  bool setIntegrationTime(int32_t integration_time) override {
    ltr329_integrationtime_t int_time;
    switch (integration_time) {
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
  bool setMeasurementRate(int32_t measurement_rate) override {
    ltr329_measurerate_t meas_rate;
    switch (measurement_rate) {
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
      @brief    Reads the LTR329's ambient light level ([Visible+IR] - IR-only)
      @param    lightEvent
                Light sensor reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!_LTR329->newDataAvailable())
      return false;

    uint16_t visible_plus_ir, infrared;
    _LTR329->readBothChannels(visible_plus_ir, infrared);
    lightEvent->light = visible_plus_ir - infrared;
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
    if (!_LTR329->newDataAvailable())
      return false;

    uint16_t visible_plus_ir, infrared;
    _LTR329->readBothChannels(visible_plus_ir, infrared);
    rawEvent->data[0] = (float)infrared;
    return true;
  }

protected:
  Adafruit_LTR329 *_LTR329; ///< Pointer to LTR329 light sensor object
};

#endif // drvLtr329_Ltr303