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

// Status codes for VL53L4CD sensor measurements
#define VL53L4CD_STATUS_VALID 0 // Returned distance is valid
#define VL53L4CD_STATUS_WARNING_SIGMA_THRESHOLD 1 // Sigma is above the defined threshold
#define VL53L4CD_STATUS_WARNING_SIGNAL_LOW 2 // Signal is below the defined threshold
#define VL53L4CD_STATUS_ERROR_DISTANCE_LOW 3 // Measured distance is below detection threshold
#define VL53L4CD_STATUS_ERROR_PHASE_OUT_OF_LIMIT 4 // Phase out of valid limit
#define VL53L4CD_STATUS_ERROR_HARDWARE_FAIL 5 // Hardware failure
#define VL53L4CD_STATUS_WARNING_NO_WRAP_CHECK 6 // Phase valid but no wrap around check performed
#define VL53L4CD_STATUS_ERROR_WRAP_TARGET 7 // Wrapped target, phase does not match
#define VL53L4CD_STATUS_ERROR_PROCESSING_FAIL 8 // Processing failure
#define VL53L4CD_STATUS_ERROR_CROSSTALK_FAIL 9 // Crosstalk signal failure
#define VL53L4CD_STATUS_ERROR_INTERRUPT 10 // Interrupt error
#define VL53L4CD_STATUS_ERROR_MERGED_TARGET 11 // Merged target detected
#define VL53L4CD_STATUS_ERROR_SIGNAL_TOO_LOW 12 // Signal is too low for accurate measurement
#define VL53L4CD_STATUS_ERROR_OTHER 255 // Other error (e.g., boot error)

const char* getVL53L4CDStatusDescription(int statusCode) {
    switch (statusCode) {
        case VL53L4CD_STATUS_VALID:
            return "Returned distance is valid";
        case VL53L4CD_STATUS_WARNING_SIGMA_THRESHOLD:
            return "Sigma is above the defined threshold";
        case VL53L4CD_STATUS_WARNING_SIGNAL_LOW:
            return "Signal is below the defined threshold";
        case VL53L4CD_STATUS_ERROR_DISTANCE_LOW:
            return "Measured distance is below detection threshold";
        case VL53L4CD_STATUS_ERROR_PHASE_OUT_OF_LIMIT:
            return "Phase out of valid limit";
        case VL53L4CD_STATUS_ERROR_HARDWARE_FAIL:
            return "Hardware failure";
        case VL53L4CD_STATUS_WARNING_NO_WRAP_CHECK:
            return "Phase valid but no wrap around check performed";
        case VL53L4CD_STATUS_ERROR_WRAP_TARGET:
            return "Wrapped target, phase does not match";
        case VL53L4CD_STATUS_ERROR_PROCESSING_FAIL:
            return "Processing failure";
        case VL53L4CD_STATUS_ERROR_CROSSTALK_FAIL:
            return "Crosstalk signal failure";
        case VL53L4CD_STATUS_ERROR_INTERRUPT:
            return "Interrupt error";
        case VL53L4CD_STATUS_ERROR_MERGED_TARGET:
            return "Merged target detected";
        case VL53L4CD_STATUS_ERROR_SIGNAL_TOO_LOW:
            return "Signal is too low for accurate measurement";
        case VL53L4CD_STATUS_ERROR_OTHER:
            return "Other error (e.g., boot error)";
        default:
            return "Unknown status";
    }
}

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
    // Program the highest possible TimingBudget, without enabling the
    // low power mode. This should give the best accuracy
    if (_VL53L4CD->VL53L4CD_SetRangeTiming(200, 0) != VL53L4CD_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to set VL53L4CD timing!");
      return false;
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
    uint8_t i = 0;
    do {
      i++;
      status = _VL53L4CD->VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (!NewDataReady || i > 50);

    if ((!status) && (NewDataReady != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      _VL53L4CD->VL53L4CD_ClearInterrupt();

      // Read measured distance. RangeStatus = 0 means valid data
      if (_VL53L4CD->VL53L4CD_GetResult(&results) == VL53L4CD_ERROR_NONE) {
        if (results.range_status != VL53L4CD_STATUS_VALID) {
          WS_DEBUG_PRINT("VL53L4CD range status: ");
          WS_DEBUG_PRINTLN(getVL53L4CDStatusDescription(results.range_status));
          return false;
        }
        proximityEvent->data[0] = (float)results.distance_mm;
        return true;
      }
      // TODO: Once I2C sensors fire all data points during a single call, we
      // can return the std deviation in MM for the measurements. See
      // https://github.com/stm32duino/VL53L4CD/blob/066664f983bcf70819133c7fcf43101035b09bab/src/vl53l4cd_api.h#L130-L131
    }
    return false;
  }

protected:
  VL53L4CD *_VL53L4CD; ///< Pointer to VL53L4CD temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L4CD