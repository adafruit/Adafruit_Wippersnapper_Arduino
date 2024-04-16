/*!
 * @file WipperSnapper_I2C_Driver_VL53L4CD.h
 *
 * Device driver for the VL53L4CD ToF sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2022 Tyeth Gundry for Adafruit Industries
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_VL53L4CD_H
#define WipperSnapper_I2C_Driver_VL53L4CD_H

#include "WipperSnapper_I2C_Driver.h"
#include <vl53l4cd_class.h>
#define VL53L4CD_STATUS_VALID 0 ///< Returned distance is valid

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L4CD sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VL53L4CD : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L4CD sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VL53L4CD(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L4CD sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VL53L4CD() {
    // Called when a VL53L4CD component is deleted.
    delete _VL53L4CD;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL53L4CD sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _VL53L4CD = new VL53L4CD(_i2c, -1);

    if (_VL53L4CD->InitSensor((uint8_t)_sensorAddress) != VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to initialize VL53L4CD sensor!");
      return false;
    }
    // Program the highest possible TimingBudget, no interval time
    if (_VL53L4CD->VL53L4CD_SetRangeTiming(200, 0) != VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to set VL53L4CD timing!");
      return false;
    }

    uint16_t signalThreshold = -1;
    if (_VL53L4CD->VL53L4CD_GetSignalThreshold(&signalThreshold) !=
        VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to get VL53L4CD signal threshold!");
    } else {
      WS_DEBUG_PRINT("VL53L4CD old signal threshold: ");
      WS_DEBUG_PRINTLN(signalThreshold);
    }
    if (_VL53L4CD->VL53L4CD_SetSignalThreshold(50) != VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to set new VL53L4CD signal threshold!");
    }

    uint16_t sigmaThreshold = -1;
    if (_VL53L4CD->VL53L4CD_GetSigmaThreshold(&sigmaThreshold) !=
        VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to get VL53L4CD sigma threshold!");
    } else {
      WS_DEBUG_PRINT("VL53L4CD old sigma threshold: ");
      WS_DEBUG_PRINTLN(sigmaThreshold);
    }
    if (_VL53L4CD->VL53L4CD_SetSigmaThreshold(100) != VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to set VL53L4CD sigma threshold!");
    }

    if (_VL53L4CD->VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to start VL53L4CD ranging!");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL53L4CD's current proximity.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    uint8_t NewDataReady = 0;
    VL53L4CD_Result_t results;
    uint8_t status;
    // Start fresh reading, seemed to be accepting stale value
    _VL53L4CD->VL53L4CD_ClearInterrupt();
    WS_DEBUG_PRINT("Waiting for VL53L4CD data ready...");
    delay(250);

    for (uint8_t retries = 0;
         (status = _VL53L4CD->VL53L4CD_CheckForDataReady(&NewDataReady)) &&
         !NewDataReady && retries < 3;
         retries++) {
      delay(300);
      WS_DEBUG_PRINT(" .");
    }
    WS_DEBUG_PRINTLN();
    if ((status == VL53L4CD_ERROR_NONE) && (NewDataReady != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      _VL53L4CD->VL53L4CD_ClearInterrupt();

      // Read measured distance. RangeStatus = 0 means valid data
      if (_VL53L4CD->VL53L4CD_GetResult(&results) == VL53L4CD_ERROR_NONE) {
        if (results.range_status != VL53L4CD_STATUS_VALID) {
          WS_DEBUG_PRINT("VL53L4CD range status: ");
          WS_DEBUG_PRINTLN(results.range_status);
          return false;
        }
        proximityEvent->data[0] = (float)results.distance_mm;
        return true;
      }
      // NOTE: Once I2C sensors fire all data points during a single call, we
      // can return the std deviation in MM for the measurements. See
      // https://github.com/stm32duino/VL53L4CD/blob/066664f983bcf70819133c7fcf43101035b09bab/src/vl53l4cd_api.h#L130-L131
    } else {
      if (status == VL53L4CD_ERROR_INVALID_ARGUMENT) {
        WS_DEBUG_PRINTLN("VL53L4CD: Invalid argument to CheckForDataReady()");
      } else if (status == VL53L4CD_ERROR_TIMEOUT) {
        WS_DEBUG_PRINTLN("VL53L4CD: Timeout waiting for data ready");
      } else {
        WS_DEBUG_PRINTLN("VL53L4CD: data not ready yet");
      }
    }
    return false;
  }

protected:
  VL53L4CD *_VL53L4CD; ///< Pointer to VL53L4CD temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L4CD