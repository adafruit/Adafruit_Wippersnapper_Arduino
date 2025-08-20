/*!
 * @file WipperSnapper_I2C_Driver_MLX90632.h
 *
 * Device driver for a Melexis MLX90632-D (medical) thermal FIR sensor.
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

#ifndef WipperSnapper_I2C_Driver_MLX90632_H
#define WipperSnapper_I2C_Driver_MLX90632_H

#include <Adafruit_MLX90632.h>

#include "WipperSnapper_I2C_Driver.h"

/**************************************************************************/
/*!
    @brief  Sensor driver for the Melexis MLX90632-D temperature sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MLX90632D : public WipperSnapper_I2C_Driver {
 public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MLX90632 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MLX90632D(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _deviceTemp = NAN;
    _objectTemp = NAN;
    _lastRead = 0;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MLX90632 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MLX90632D() { delete _mlx90632; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MLX90632 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _mlx90632 = new Adafruit_MLX90632();
    // attempt to initialize MLX90632
    if (!_mlx90632->begin(_sensorAddress, _i2c))
      return false;

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
    WS_DEBUG_PRINTLN(F("Device reset: SUCCESS"));

    uint64_t productID = _mlx90632->getProductID();
    WS_DEBUG_PRINT(F("Product ID: 0x"));
    WS_DEBUG_PRINT((uint32_t)(productID >> 32), HEX);
    WS_DEBUG_PRINTLN((uint32_t)(productID & 0xFFFFFFFF), HEX);

    uint16_t productCode = _mlx90632->getProductCode();
    WS_DEBUG_PRINT(F("Product Code: 0x"));
    WS_DEBUG_PRINTLN(productCode, HEX);

    uint16_t eepromVersion = _mlx90632->getEEPROMVersion();
    WS_DEBUG_PRINT(F("EEPROM Version: 0x"));
    WS_DEBUG_PRINTLN(eepromVersion, HEX);

    // Decode product code bits
    uint8_t fov = (productCode >> 8) & 0x3;
    uint8_t package = (productCode >> 5) & 0x7;
    uint8_t accuracy = productCode & 0x1F;

    WS_DEBUG_PRINT(F("FOV: "));
    WS_DEBUG_PRINTLN(fov == 0 ? F("50°") : F("Unknown"));

    WS_DEBUG_PRINT(F("Package: "));
    WS_DEBUG_PRINTLN(package == 1 ? F("SFN 3x3") : F("Unknown"));

    WS_DEBUG_PRINT(F("Accuracy: "));
    if (accuracy == 1) {
      WS_DEBUG_PRINTLN(F("Medical"));
    } else if (accuracy == 2) {
      WS_DEBUG_PRINTLN(F("Standard"));
    } else {
      WS_DEBUG_PRINTLN(F("Unknown"));
    }

    // Set and get mode - choose one:
    WS_DEBUG_PRINTLN(F("\n--- Mode Settings ---"));
    if (!_mlx90632->setMode(MLX90632_MODE_CONTINUOUS)) {
      // if (!_mlx90632->setMode(MLX90632_MODE_STEP)) {           // Uncomment
      // for step mode testing if
      // (!_mlx90632->setMode(MLX90632_MODE_SLEEPING_STEP)) {  // Uncomment for
      // sleeping step mode testing
      WS_DEBUG_PRINTLN(F("Failed to set mode"));
      while (1) {
        delay(10);
      }
    }

    // TODO: use Step mode?
    mlx90632_mode_t currentMode = _mlx90632->getMode();
    WS_DEBUG_PRINT(F("Current mode: "));
    switch (currentMode) {
      case MLX90632_MODE_HALT:
        WS_DEBUG_PRINTLN(F("Halt"));
        break;
      case MLX90632_MODE_SLEEPING_STEP:
        WS_DEBUG_PRINTLN(F("Sleeping Step"));
        break;
      case MLX90632_MODE_STEP:
        WS_DEBUG_PRINTLN(F("Step"));
        break;
      case MLX90632_MODE_CONTINUOUS:
        WS_DEBUG_PRINTLN(F("Continuous"));
        break;
      default:
        WS_DEBUG_PRINTLN(F("Unknown"));
    }

    // set accuracy mode based on medical if detected
    if (accuracy == 1) {
      // Set and get measurement select (medical)
      WS_DEBUG_PRINTLN(F("\n--- Measurement Select Settings ---"));
      if (!extendedInsteadOfMedicalRange &&
          !_mlx90632->setMeasurementSelect(MLX90632_MEAS_MEDICAL)) {
        WS_DEBUG_PRINTLN(F("Failed to set measurement select to Medical"));
        while (1) {
          delay(10);
        }
      } else if (extendedInsteadOfMedicalRange &&
                 !_mlx90632->setMeasurementSelect(
                     MLX90632_MEAS_EXTENDED_RANGE)) {
        WS_DEBUG_PRINTLN(
            F("Failed to set measurement select to Extended Range"));
        while (1) {
          delay(10);
        }
      }

      mlx90632_meas_select_t currentMeasSelect =
          _mlx90632->getMeasurementSelect();
      WS_DEBUG_PRINT(F("Current measurement select: "));
      switch (currentMeasSelect) {
        case MLX90632_MEAS_MEDICAL:
          WS_DEBUG_PRINTLN(F("Medical"));
          break;
        case MLX90632_MEAS_EXTENDED_RANGE:
          WS_DEBUG_PRINTLN(F("Extended Range"));
          break;
        default:
          WS_DEBUG_PRINTLN(F("Unknown"));
      }
    }

    // Set and get refresh rate (default to 2Hz)
    WS_DEBUG_PRINTLN(F("\n--- Refresh Rate Settings ---"));
    if (!_mlx90632->setRefreshRate(MLX90632_REFRESH_2HZ)) {
      WS_DEBUG_PRINTLN(F("Failed to set refresh rate to 2Hz"));
      while (1) {
        delay(10);
      }
    }

    mlx90632_refresh_rate_t currentRefreshRate = _mlx90632->getRefreshRate();
    WS_DEBUG_PRINT(F("Current refresh rate: "));
    switch (currentRefreshRate) {
      case MLX90632_REFRESH_0_5HZ:
        WS_DEBUG_PRINTLN(F("0.5 Hz"));
        break;
      case MLX90632_REFRESH_1HZ:
        WS_DEBUG_PRINTLN(F("1 Hz"));
        break;
      case MLX90632_REFRESH_2HZ:
        WS_DEBUG_PRINTLN(F("2 Hz"));
        break;
      case MLX90632_REFRESH_4HZ:
        WS_DEBUG_PRINTLN(F("4 Hz"));
        break;
      case MLX90632_REFRESH_8HZ:
        WS_DEBUG_PRINTLN(F("8 Hz"));
        break;
      case MLX90632_REFRESH_16HZ:
        WS_DEBUG_PRINTLN(F("16 Hz"));
        break;
      case MLX90632_REFRESH_32HZ:
        WS_DEBUG_PRINTLN(F("32 Hz"));
        break;
      case MLX90632_REFRESH_64HZ:
        WS_DEBUG_PRINTLN(F("64 Hz"));
        break;
      default:
        WS_DEBUG_PRINTLN(F("Unknown"));
    }

    // Clear new data flag before starting continuous measurements
    WS_DEBUG_PRINTLN(F("\\n--- Starting Continuous Measurements ---"));
    if (!_mlx90632->resetNewData()) {
      WS_DEBUG_PRINTLN(F("Failed to reset new data flag"));
      while (1) {
        delay(10);
      }
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
    return false;  // sensor not read recently, return false
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
    return false;  // sensor not read recently, return false
  }

 protected:
  double _deviceTemp;  ///< Device temperature in Celsius
  double _objectTemp;  ///< Object temperature in Celsius
  uint32_t _lastRead;  ///< Last time the sensor was read in milliseconds
  Adafruit_MLX90632 *_mlx90632 = nullptr;  ///< MLX90632 object
};

#endif  // WipperSnapper_I2C_Driver_MLX90632