/*!
 * @file drvLps28dfw.h
 *
 * Device driver for a LPS28DFW precision pressure sensor breakout.
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

#ifndef DRV_LPS28DFW_H
#define DRV_LPS28DFW_H

#include "drvBase.h"
#include <Adafruit_LPS28.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS28DFW temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvLps28dfw : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS28DFW sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvLps28dfw(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS28DFW sensor.
  */
  /*******************************************************************************/
  ~drvLps28dfw() { delete _lps28; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS28DFW sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _lps28 = new Adafruit_LPS28();
    // attempt to initialize LPS28DFW
    if (!_lps28->begin(_i2c, _address))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the LPS28DFW sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    return _lps28->setDataRate(LPS28_ODR_ONESHOT) &&
           _lps28->setAveraging(LPS28_AVG_512) &&
           _lps28->setFullScaleMode(true);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the output data rate (ODR) setting to the driver. The
                ODR selects the sensor's sampling frequency, in Hz.
      @param    output_data_rate
                The output data rate index from the broker
                (0=One-shot, 1=1Hz, 2=4Hz, 3=10Hz, 4=25Hz, 5=50Hz, 6=75Hz,
                7=100Hz, 8=200Hz).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setOutputDataRate(const ws_config_Value &output_data_rate) override {
    if (output_data_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    lps28_odr_t odr;
    switch (output_data_rate.value.int_value) {
    case 0:
      odr = LPS28_ODR_ONESHOT;
      break;
    case 1:
      odr = LPS28_ODR_1_HZ;
      break;
    case 2:
      odr = LPS28_ODR_4_HZ;
      break;
    case 3:
      odr = LPS28_ODR_10_HZ;
      break;
    case 4:
      odr = LPS28_ODR_25_HZ;
      break;
    case 5:
      odr = LPS28_ODR_50_HZ;
      break;
    case 6:
      odr = LPS28_ODR_75_HZ;
      break;
    case 7:
      odr = LPS28_ODR_100_HZ;
      break;
    case 8:
      odr = LPS28_ODR_200_HZ;
      break;
    default:
      return false;
    }
    return _lps28->setDataRate(odr);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the averaged-samples setting to the driver. Averaging
                trades noise for conversion time.
      @param    averaged_samples
                The averaging index from the broker
                (0=4, 1=8, 2=16, 3=32, 4=64, 5=128, 6=512 samples).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setAveragedSamples(const ws_config_Value &averaged_samples) override {
    if (averaged_samples.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    lps28_avg_t avg;
    switch (averaged_samples.value.int_value) {
    case 0:
      avg = LPS28_AVG_4;
      break;
    case 1:
      avg = LPS28_AVG_8;
      break;
    case 2:
      avg = LPS28_AVG_16;
      break;
    case 3:
      avg = LPS28_AVG_32;
      break;
    case 4:
      avg = LPS28_AVG_64;
      break;
    case 5:
      avg = LPS28_AVG_128;
      break;
    case 6:
      avg = LPS28_AVG_512;
      break;
    default:
      return false;
    }
    return _lps28->setAveraging(avg);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the full-scale pressure range setting to the driver.
      @param    range
                The range index from the broker
                (0=1260 hPa higher resolution, 1=4060 hPa wider range).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setRange(const ws_config_Value &range) override {
    if (range.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    switch (range.value.int_value) {
    case 0:
      return _lps28->setFullScaleMode(false);
    case 1:
      return _lps28->setFullScaleMode(true);
    default:
      return false;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the low-pass filter setting to the driver.
      @param    filter
                The filter index from the broker
                (0=Off, 1=On / ODR divided by 9).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setFilter(const ws_config_Value &filter) override {
    if (filter.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    switch (filter.value.int_value) {
    case 0:
      return _lps28->setLowPassODR9(false);
    case 1:
      return _lps28->setLowPassODR9(true);
    default:
      return false;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Triggers one one-shot conversion and reads temperature and
                pressure together once both data-ready bits (P_DA and T_DA)
                are set, so the pass gets one consistent sample instead of
                two conversions.
      @returns  True if the sample was read, False on trigger failure or if
                the conversion did not complete within 100ms.
  */
  /*******************************************************************************/
  bool ReadSensorData() override {
    if (!_lps28->triggerOneShot())
      return false;
    const uint8_t both = LPS28_STATUS_PRESS_READY | LPS28_STATUS_TEMP_READY;
    for (uint8_t i = 0; i < 100; i++) {
      if ((_lps28->getStatus() & both) == both)
        return _lps28->getEvent(&_pressure_ev, &_temp_ev);
      delay(1);
    }
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the LPS28DFW's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!AttemptRead())
      return false;
    tempEvent->temperature = _temp_ev.temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    if (!AttemptRead())
      return false;
    pressureEvent->pressure = _pressure_ev.pressure;
    return true;
  }

protected:
  Adafruit_LPS28 *_lps28 = nullptr;   ///< LPS28DFW  object
  sensors_event_t _temp_ev = {0};     ///< Cached temperature event
  sensors_event_t _pressure_ev = {0}; ///< Cached pressure event
};

#endif // DRV_LPS28DFW_H
