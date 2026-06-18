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

    return ConfigureAndPrintSensorInfo();
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the MLX90632 sensor and prints its information.
      @param    extendedInsteadOfMedicalRange
                If true, configures the sensor for extended temperature
     range/acc.
      @returns  True if configuration fetching and setting were successful.

      CUSTOM SETTINGS CANDIDATES (currently chosen automatically / hard-coded;
      not yet exposed via the v2 properties API):
       - Measurement select: MEDICAL (default, higher accuracy ~ body-temp
         range) vs EXTENDED_RANGE (wider temperature span). Surfaced today only
         via the "mlx90632d_ext" driver key / this argument.
       - Measurement mode: CONTINUOUS vs STEP / SLEEPING_STEP (lower power,
         on-demand).
       - Refresh rate: setRefreshRate(MLX90632_REFRESH_0_5HZ..64HZ) trades
         update rate / noise for power.
       - Emissivity (if exposed by the library) for non-blackbody targets.
  */
  /*******************************************************************************/
  bool ConfigureAndPrintSensorInfo(bool extendedInsteadOfMedicalRange = false) {
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
      if (!extendedInsteadOfMedicalRange &&
          !_mlx90632->setMeasurementSelect(MLX90632_MEAS_MEDICAL)) {
        WS_DEBUG_PRINTLN("Failed to set measurement select to Medical");
        return false;
      } else if (extendedInsteadOfMedicalRange &&
                 !_mlx90632->setMeasurementSelect(
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
  double _deviceTemp;           ///< Device temperature in Celsius
  double _objectTemp;           ///< Object temperature in Celsius
  Adafruit_MLX90632 *_mlx90632; ///< Pointer to MLX90632 sensor object
};

#endif // DRV_MLX90632_H
