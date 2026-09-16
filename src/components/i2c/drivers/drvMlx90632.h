/*!
 * @file drvMlx90632.h
 *
 * Device driver for the Melexis MLX90632 Far Infrared temp sensor
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
#ifndef DRV_MLX90632_H
#define DRV_MLX90632_H

#include "drvBase.h"
#include <Adafruit_MLX90632.h>

#define MLX90632_TICK_MS 50 ///< Poll for a finished measurement every 50ms

/*!
    @brief  Class that provides a driver interface for an MLX90632 sensor.
*/
class drvMLX90632 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MLX90632 sensor.
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
  drvMLX90632(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // A measurement takes up to seconds at the slow refresh rates (and must be
    // requested explicitly in step modes), so it is collected by fastTick()
    // during the lead window before each read is due rather than blocking the
    // read pass. The lead is set from the configured mode and rate.
    _fast_tick_ms = MLX90632_TICK_MS;
    _mlx90632 = nullptr;
    _deviceTemp = NAN;
    _objectTemp = NAN;
    _extendedRange = false;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MLX90632 sensor.
  */
  /*******************************************************************************/
  ~drvMLX90632() {
    if (_mlx90632) {
      delete _mlx90632;
      _mlx90632 = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MLX90632 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_mlx90632) {
      delete _mlx90632;
      _mlx90632 = nullptr;
    }
    _mlx90632 = new Adafruit_MLX90632();
    if (!_mlx90632->begin(_address, _i2c)) {
      return false;
    }

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Selects the extended temperature range variant (vs. the default
                medical range). Records the choice so it is applied by
                configureDefaults() once the transport has been initialized.
      @param    extendedInsteadOfMedicalRange
                If true, configures the sensor for extended temperature
     range/acc.
  */
  /*******************************************************************************/
  void ConfigureAndPrintSensorInfo(bool extendedInsteadOfMedicalRange = false) {
    _extendedRange = extendedInsteadOfMedicalRange;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the MLX90632 sensor with default settings.
      @returns  True if configuration fetching and setting were successful.

      The measurement-select (MEDICAL vs EXTENDED_RANGE) is chosen by the driver
      variant ("mlx90632d_ext" driver key, recorded via
      ConfigureAndPrintSensorInfo) and is intentionally NOT exposed as a
      broker-settable property.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    // Reset the device
    if (!_mlx90632->reset()) {
      WS_DEBUG_PRINTLN("Device reset failed");
      return false;
    }

    uint16_t productCode = _mlx90632->getProductCode();
    // Decode product code bits
    uint8_t accuracy = productCode & 0x1F;

    if (!_mlx90632->setMode(MLX90632_MODE_CONTINUOUS)) {
      WS_DEBUG_PRINTLN("Failed to set mode");
      return false;
    }

    // set accuracy mode based on medical if detected
    if (accuracy == 1) {
      // Set and get measurement select (medical)
      if (!_extendedRange &&
          !_mlx90632->setMeasurementSelect(MLX90632_MEAS_MEDICAL)) {
        WS_DEBUG_PRINTLN("Failed to set measurement select to Medical");
        return false;
      } else if (_extendedRange && !_mlx90632->setMeasurementSelect(
                                       MLX90632_MEAS_EXTENDED_RANGE)) {
        WS_DEBUG_PRINTLN("Failed to set measurement select to Extended Range");
        return false;
      }
    }

    // Set and get refresh rate (default to 2Hz)
    if (!_mlx90632->setRefreshRate(MLX90632_REFRESH_2HZ)) {
      WS_DEBUG_PRINTLN("Failed to set refresh rate to 2Hz");
      return false;
    }

    if (!_mlx90632->resetNewData()) {
      WS_DEBUG_PRINTLN("Failed to reset new data flag");
      return false;
    }
    _mode = MLX90632_MODE_CONTINUOUS;
    _tick_lead_ms = 2 * (uint32_t)getRefreshDelay();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the measurement mode setting to the driver. Step and
                sleeping-step modes lower power by measuring on demand.
      @param    mode
                The mode index from the broker
                (0=Continuous, 1=Step, 2=Sleeping Step, 3=Halt).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMode(const ws_config_Value &mode) override {
    if (mode.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    mlx90632_mode_t devMode;
    switch (mode.value.int_value) {
    case 0:
      devMode = MLX90632_MODE_CONTINUOUS;
      break;
    case 1:
      devMode = MLX90632_MODE_STEP;
      break;
    case 2:
      devMode = MLX90632_MODE_SLEEPING_STEP;
      break;
    case 3:
      devMode = MLX90632_MODE_HALT;
      break;
    default:
      return false;
    }
    if (!_mlx90632->setMode(devMode))
      return false;
    _mode = devMode;
    _tick_lead_ms = 2 * (uint32_t)getRefreshDelay();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the measurement (refresh) rate setting to the driver.
                Higher rates trade power/noise for faster updates.
      @param    measurement_rate
                The refresh rate index from the broker
                (0=0.5Hz, 1=1Hz, 2=2Hz, 3=4Hz, 4=8Hz, 5=16Hz, 6=32Hz, 7=64Hz).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMeasurementRate(const ws_config_Value &measurement_rate) override {
    if (measurement_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    mlx90632_refresh_rate_t rate;
    switch (measurement_rate.value.int_value) {
    case 0:
      rate = MLX90632_REFRESH_0_5HZ;
      break;
    case 1:
      rate = MLX90632_REFRESH_1HZ;
      break;
    case 2:
      rate = MLX90632_REFRESH_2HZ;
      break;
    case 3:
      rate = MLX90632_REFRESH_4HZ;
      break;
    case 4:
      rate = MLX90632_REFRESH_8HZ;
      break;
    case 5:
      rate = MLX90632_REFRESH_16HZ;
      break;
    case 6:
      rate = MLX90632_REFRESH_32HZ;
      break;
    case 7:
      rate = MLX90632_REFRESH_64HZ;
      break;
    default:
      return false;
    }
    if (!_mlx90632->setRefreshRate(rate))
      return false;
    _tick_lead_ms = 2 * (uint32_t)getRefreshDelay();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Background measurement step, called every MLX90632_TICK_MS
                while a read is pending. In step and sleeping-step modes a
                single measurement is requested first (and re-requested if it
                does not complete within the lead time); in continuous mode
                the device measures on its own. Once new data is flagged, both
                temperatures are read together, the flag cleared, and a valid
                sample filed with NewSample().
  */
  /*******************************************************************************/
  void fastTick() override {
    if (!_mlx90632->isNewData()) {
      bool step =
          (_mode == MLX90632_MODE_STEP || _mode == MLX90632_MODE_SLEEPING_STEP);
      // Step modes measure only on request: kick one off if none in flight
      if (step && (!_measuring || millis() - _measure_start > _tick_lead_ms)) {
        if (_mlx90632->startSingleMeasurement()) {
          _measuring = true;
          _measure_start = millis();
        } else {
          WS_DEBUG_PRINTLN("Failed to start single measurement");
        }
      }
      return;
    }
    _measuring = false;

    double deviceTemp = _mlx90632->getAmbientTemperature();
    double objectTemp = _mlx90632->getObjectTemperature();
    // Reset new data flag after reading so the next check is genuine
    if (!_mlx90632->resetNewData()) {
      WS_DEBUG_PRINTLN("Failed to reset new data flag");
    }
    if (isnan(objectTemp)) {
      WS_DEBUG_PRINTLN("NaN (invalid cycle position)");
      return;
    }
    _deviceTemp = deviceTemp;
    _objectTemp = objectTemp;
    NewSample();
  }

  /*******************************************************************************/
  /*!
      @brief Calculates the maximum wait time in milliseconds for a new
     measurement to be ready based on the current mode and refresh rate.
      @return The maximum wait time in milliseconds.
      @note See Melexis App Note "MLX90632 measurement modes" tables 10+11.
  */
  /*******************************************************************************/
  int16_t getRefreshDelay() {
    int16_t maxWaitTimeMs = 0;
    // In Normal step/continous mode (std/medical), Standard
    // rates are 1/2 of Burst rates (step/sleep_step), but
    // in Extended mode all times are same for Step/Continous
    mlx90632_mode_t mode = _mlx90632->getMode();
    bool extended =
        (_mlx90632->getMeasurementSelect() == MLX90632_MEAS_EXTENDED_RANGE);

    switch (_mlx90632->getRefreshRate()) {
    case MLX90632_REFRESH_0_5HZ:
      maxWaitTimeMs = extended                           ? 6000
                      : mode == MLX90632_MODE_CONTINUOUS ? 2000
                                                         : 4000;
      break;
    case MLX90632_REFRESH_1HZ:
      maxWaitTimeMs = extended                           ? 3000
                      : mode == MLX90632_MODE_CONTINUOUS ? 1000
                                                         : 2000;
      break;
    case MLX90632_REFRESH_2HZ:
      maxWaitTimeMs = extended                           ? 1500
                      : mode == MLX90632_MODE_CONTINUOUS ? 500
                                                         : 1000;
      break;
    case MLX90632_REFRESH_4HZ:
      maxWaitTimeMs = extended                           ? 750
                      : mode == MLX90632_MODE_CONTINUOUS ? 250
                                                         : 500;
      break;
    case MLX90632_REFRESH_8HZ:
      maxWaitTimeMs = extended                           ? 375
                      : mode == MLX90632_MODE_CONTINUOUS ? 125
                                                         : 250;
      break;
    case MLX90632_REFRESH_16HZ:
      maxWaitTimeMs = extended                           ? 200
                      : mode == MLX90632_MODE_CONTINUOUS ? 63
                                                         : 125;
      break;
    case MLX90632_REFRESH_32HZ:
      maxWaitTimeMs = extended                           ? 100
                      : mode == MLX90632_MODE_CONTINUOUS ? 32
                                                         : 63;
      break;
    case MLX90632_REFRESH_64HZ:
      maxWaitTimeMs = extended                           ? 50
                      : mode == MLX90632_MODE_CONTINUOUS ? 16
                                                         : 32;
      break;
    default:
      WS_DEBUG_PRINTLN("MLX90632: Unknown refresh rate");
      maxWaitTimeMs = 510; // Default to 510ms (old value)
    }
    return maxWaitTimeMs;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MLX90632's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!AttemptRead() || isnan(_deviceTemp))
      return false;
    tempEvent->temperature = _deviceTemp;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MLX90632's object temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventObjectTemp(sensors_event_t *tempEvent) {
    if (!AttemptRead() || isnan(_objectTemp))
      return false;
    tempEvent->temperature = _objectTemp;
    return true;
  }

protected:
  double _deviceTemp = NAN; ///< Device temperature in Celsius
  double _objectTemp = NAN; ///< Object temperature in Celsius
  bool _extendedRange; ///< True for extended-range variant, false for medical
  mlx90632_mode_t _mode = MLX90632_MODE_CONTINUOUS; ///< Configured mode
  bool _measuring = false;  ///< Step modes: a single measurement is in flight
  ulong _measure_start = 0; ///< millis() the in-flight measurement started
  Adafruit_MLX90632 *_mlx90632; ///< Pointer to MLX90632 sensor object
};

#endif // DRV_MLX90632_H
