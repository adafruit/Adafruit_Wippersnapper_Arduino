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
    return _mlx90632->setMode(devMode);
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
    return _mlx90632->setRefreshRate(rate);
  }

  /*******************************************************************************/
  /*!
      @brief    Reads ambient and object temperatures together when the sensor
                has new data ready. Leaves the cached members untouched (last
                good sample) when no new data is available this pass.
      @returns  True if a fresh sample was read, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    bool result = false;

    // Check if we need to trigger a new measurement for step modes
    mlx90632_mode_t currentMode = _mlx90632->getMode();
    if (currentMode == MLX90632_MODE_STEP ||
        currentMode == MLX90632_MODE_SLEEPING_STEP) {
      // Trigger single measurement (SOC bit) for step modes
      if (!_mlx90632->startSingleMeasurement()) {
        WS_DEBUG_PRINTLN("Failed to start single measurement");
        return false;
      }
      delay(510); // Wait for measurement to complete @ 2Hz
    }

    // Only check new data flag - much more efficient for continuous mode
    if (_mlx90632->isNewData()) {
      _deviceTemp = _mlx90632->getAmbientTemperature();
      _objectTemp = _mlx90632->getObjectTemperature();
      if (isnan(_objectTemp)) {
        WS_DEBUG_PRINTLN("NaN (invalid cycle position)");
        return false;
      }
      result = true;
      // Reset new data flag after reading
      if (!_mlx90632->resetNewData()) {
        WS_DEBUG_PRINTLN("Failed to reset new data flag");
      }
    } else {
      WS_DEBUG_PRINTLN("No new data available, skipping read");
    }

    return result;
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
    // Refresh both temps together; the member holds the last good sample if no
    // new data was ready this pass.
    ReadSensorData();
    if (isnan(_deviceTemp))
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
    ReadSensorData();
    if (isnan(_objectTemp))
      return false;
    tempEvent->temperature = _objectTemp;
    return true;
  }

protected:
  double _deviceTemp;  ///< Device temperature in Celsius
  double _objectTemp;  ///< Object temperature in Celsius
  bool _extendedRange; ///< True for extended-range variant, false for medical
  Adafruit_MLX90632 *_mlx90632; ///< Pointer to MLX90632 sensor object
};

#endif // DRV_MLX90632_H
