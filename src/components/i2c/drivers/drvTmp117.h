/*!
 * @file drvTmp117.h
 *
 * Device driver for the TMP117 Temperature sensor.
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
#ifndef DRV_TMP117_H
#define DRV_TMP117_H

#include "drvBase.h"
#include <Adafruit_TMP117.h>

/*!
    @brief  Class that provides a driver interface for a TMP117 sensor.
*/
class drvTmp117 : public drvBase {
public:
  /*!
      @brief    Constructor for a TMP117 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvTmp117(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an TMP117 sensor.
  */
  ~drvTmp117() {
    // Called when a TMP117 component is deleted.
    delete _tmp117;
  }

  /*!
      @brief    Initializes the TMP117 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _tmp117 = new Adafruit_TMP117();
    return _tmp117->begin((uint8_t)_address, _i2c);
  }

  /*!
      @brief    Configures the TMP117 sensor with default settings. The
                TMP117 powers up in its default configuration, so no
                additional setup is required here.
      @returns  True if configured successfully, False otherwise.
  */
  bool configureDefaults() override { return true; }

  /*!
      @brief    Applies the averaged sample count setting to the driver.
      @param    averaged_samples
                The averaged sample count index from the broker
                (0=1x, 1=8x, 2=32x, 3=64x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setAveragedSamples(const ws_config_Value &averaged_samples) override {
    if (averaged_samples.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = averaged_samples.value.int_value;
    tmp117_average_count_t count;
    switch (val) {
    case 0:
      count = TMP117_AVERAGE_1X;
      break;
    case 1:
      count = TMP117_AVERAGE_8X;
      break;
    case 2:
      count = TMP117_AVERAGE_32X;
      break;
    case 3:
      count = TMP117_AVERAGE_64X;
      break;
    default:
      count = TMP117_AVERAGE_1X;
      break;
    }
    return _tmp117->setAveragedSampleCount(count);
  }

  /*!
      @brief    Applies the read delay setting to the driver. The read delay
                is the minimum delay between new measurements.
      @param    read_delay
                The read delay index from the broker
                (0=0ms, 1=125ms, 2=250ms, 3=500ms, 4=1000ms, 5=4000ms,
                6=8000ms, 7=16000ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setReadDelay(const ws_config_Value &read_delay) override {
    if (read_delay.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = read_delay.value.int_value;
    tmp117_delay_t delay;
    switch (val) {
    case 0:
      delay = TMP117_DELAY_0_MS;
      break;
    case 1:
      delay = TMP117_DELAY_125_MS;
      break;
    case 2:
      delay = TMP117_DELAY_250_MS;
      break;
    case 3:
      delay = TMP117_DELAY_500_MS;
      break;
    case 4:
      delay = TMP117_DELAY_1000_MS;
      break;
    case 5:
      delay = TMP117_DELAY_4000_MS;
      break;
    case 6:
      delay = TMP117_DELAY_8000_MS;
      break;
    case 7:
      delay = TMP117_DELAY_16000_MS;
      break;
    default:
      delay = TMP117_DELAY_0_MS;
      break;
    }
    return _tmp117->setReadDelay(delay);
  }

  /*!
      @brief    Applies the measurement mode setting to the driver.
      @param    mode
                The mode index from the broker
                (0=Continuous, 1=Shutdown, 2=One Shot).
      @returns  True if applied successfully, False otherwise.
  */
  bool setMode(const ws_config_Value &mode) override {
    if (mode.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = mode.value.int_value;
    tmp117_mode_t meas_mode;
    switch (val) {
    case 0:
      meas_mode = TMP117_MODE_CONTINUOUS;
      break;
    case 1:
      meas_mode = TMP117_MODE_SHUTDOWN;
      break;
    case 2:
      meas_mode = TMP117_MODE_ONE_SHOT;
      break;
    default:
      meas_mode = TMP117_MODE_CONTINUOUS;
      break;
    }
    return _tmp117->setMeasurementMode(meas_mode);
  }

  /*!
      @brief    Gets the TMP117's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _tmp117->getEvent(tempEvent);
  }

protected:
  Adafruit_TMP117 *_tmp117; ///< Pointer to TMP117 temperature sensor object
};

#endif // drvTmp117