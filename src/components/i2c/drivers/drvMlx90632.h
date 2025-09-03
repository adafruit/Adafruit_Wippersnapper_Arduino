/*!
 * @file drvMlx90632.h
 *
 * Device driver for the Melexis MLX90632 Far Infrared temp sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
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
    _lastRead = 0;
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
    if (!_mlx90632->begin(_sensorAddress, _i2c)){
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
  */
  /*******************************************************************************/
  bool ConfigureAndPrintSensorInfo(bool extendedInsteadOfMedicalRange = false) {
    // Reset the device
    if (!_mlx90632->reset()) {
      WS_DEBUG_PRINTLN(F("Device reset failed"));
      return false;
    }

    uint16_t productCode = _mlx90632->getProductCode();
    // Decode product code bits
    uint8_t accuracy = productCode & 0x1F;

    if (!_mlx90632->setMode(MLX90632_MODE_CONTINUOUS)) {
      WS_DEBUG_PRINTLN(F("Failed to set mode"));
      return false;
    }

    // set accuracy mode based on medical if detected
    if (accuracy == 1) {
      // Set and get measurement select (medical)
      if (!extendedInsteadOfMedicalRange &&
          !_mlx90632->setMeasurementSelect(MLX90632_MEAS_MEDICAL)) {
        WS_DEBUG_PRINTLN(F("Failed to set measurement select to Medical"));
        return false;
      } else if (extendedInsteadOfMedicalRange &&
                 !_mlx90632->setMeasurementSelect(
                     MLX90632_MEAS_EXTENDED_RANGE)) {
        WS_DEBUG_PRINTLN(
            F("Failed to set measurement select to Extended Range"));
        return false;
      }
    }

    // Set and get refresh rate (default to 2Hz)
    if (!_mlx90632->setRefreshRate(MLX90632_REFRESH_2HZ)) {
      WS_DEBUG_PRINTLN(F("Failed to set refresh rate to 2Hz"));
      return false;
    }

    if (!_mlx90632->resetNewData()) {
      WS_DEBUG_PRINTLN(F("Failed to reset new data flag"));
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  /*******************************************************************************/
  bool HasBeenReadInLast200ms() {
    return _lastRead != 0 && millis() - _lastRead < 200;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    bool result = false;
    if (HasBeenReadInLast200ms()) {
      WS_DEBUG_PRINTLN(F("Sensor was read recently, using cached data"));
      return true;
    }

    // Check if we need to trigger a new measurement for step modes
    mlx90632_mode_t currentMode = _mlx90632->getMode();
    if (currentMode == MLX90632_MODE_STEP ||
        currentMode == MLX90632_MODE_SLEEPING_STEP) {
      // Trigger single measurement (SOC bit) for step modes
      if (!_mlx90632->startSingleMeasurement()) {
        WS_DEBUG_PRINTLN(F("Failed to start single measurement"));
        return false;
      }
      delay(510); // Wait for measurement to complete @ 2Hz
    }

    // Only check new data flag - much more efficient for continuous mode
    if (_mlx90632->isNewData()) {
      _deviceTemp = _mlx90632->getAmbientTemperature();
      _objectTemp = _mlx90632->getObjectTemperature();
      if (isnan(_objectTemp)) {
        WS_DEBUG_PRINTLN(F("NaN (invalid cycle position)"));
        return false;
      }
      result = true;
      _lastRead = millis();
      // Reset new data flag after reading
      if (!_mlx90632->resetNewData()) {
        WS_DEBUG_PRINTLN(F("Failed to reset new data flag"));
      }
    } else {
      WS_DEBUG_PRINTLN(F("No new data available, skipping read"));
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
    if (ReadSensorData() && _deviceTemp != NAN) {
      tempEvent->temperature = _deviceTemp;
      return true;
    }
    return false; // sensor not read recently, return false
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
    if (ReadSensorData() && _objectTemp != NAN) {
      tempEvent->temperature = _objectTemp;
      return true;
    }
    return false; // sensor not read recently, return false
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 4;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE;
    _default_sensor_types[2] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[3] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT;
  }

protected:
  double _deviceTemp; ///< Device temperature in Celsius
  double _objectTemp; ///< Object temperature in Celsius
  uint32_t _lastRead; ///< Last time the sensor was read in milliseconds
  Adafruit_MLX90632 *_mlx90632; ///< Pointer to MLX90632 sensor object
};

#endif // drvMLX90632