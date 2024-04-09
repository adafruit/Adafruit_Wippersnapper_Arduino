/*!
 * @file WipperSnapper_I2C_Driver_VL53L4CX.h
 *
 * Device driver for the VL53L4CX ToF sensor.
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
#ifndef WipperSnapper_I2C_Driver_VL53L4CX_H
#define WipperSnapper_I2C_Driver_VL53L4CX_H

#include "WipperSnapper_I2C_Driver.h"
#include <vl53l4cx_class.h>
#include <vl53l4cx_def.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L4CX sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VL53L4CX : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L4CX sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VL53L4CX(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L4CX sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VL53L4CX() {
    // Called when a VL53L4CX component is deleted.
    delete _VL53L4CX;
  }

  /**************************************************************************/
  /*!
      @brief    Gets a human-readable description of a VL53L4CX status code.
      @param    statusCode
                The status code to describe.
      @returns  A human-readable description of the status code.
  */
  const char *getVL53L4CXStatusDescription(int statusCode) {
    switch (statusCode) {
    case VL53L4CX_RANGESTATUS_RANGE_VALID:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID";
    case VL53L4CX_RANGESTATUS_SIGMA_FAIL:
      return "VL53L4CX_RANGESTATUS_SIGMA_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED";
    case VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL:
      return "VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL";
    case VL53L4CX_RANGESTATUS_HARDWARE_FAIL:
      return "VL53L4CX_RANGESTATUS_HARDWARE_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL";
    case VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL:
      return "VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL";
    case VL53L4CX_RANGESTATUS_PROCESSING_FAIL:
      return "VL53L4CX_RANGESTATUS_PROCESSING_FAIL";
    case VL53L4CX_RANGESTATUS_XTALK_SIGNAL_FAIL:
      return "VL53L4CX_RANGESTATUS_XTALK_SIGNAL_FAIL";
    case VL53L4CX_RANGESTATUS_SYNCRONISATION_INT:
      return "VL53L4CX_RANGESTATUS_SYNCRONISATION_INT";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE";
    case VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
      return "VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL";
    case VL53L4CX_RANGESTATUS_MIN_RANGE_FAIL:
      return "VL53L4CX_RANGESTATUS_MIN_RANGE_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_INVALID:
      return "VL53L4CX_RANGESTATUS_RANGE_INVALID";
    case VL53L4CX_RANGESTATUS_NONE:
      return "VL53L4CX_RANGESTATUS_NONE";
    default:
      return (("UNKNOWN STATUS: ") + String(statusCode)).c_str();
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL53L4CX sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _VL53L4CX = new VL53L4CX(_i2c, -1);

    /*   Reinstate if shutdown pin utilised
     * // Configure VL53L4CX satellite component.
     * _VL53L4CX->begin();
     *
     * // Switch off VL53L4CX satellite component.
     * _VL53L4CX->VL53L4CX_Off();
     */

    if (_VL53L4CX->InitSensor((uint8_t)_sensorAddress) != VL53L4CX_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to initialize VL53L4CX sensor!");
      return false;
    }
    // // Program the highest possible TimingBudget, no interval time
    // if (_VL53L4CX-> VL53L4CX_SetRangeTiming(200, 0) != VL53L4CX_ERROR_NONE) {
    //   WS_DEBUG_PRINTLN("Failed to set VL53L4CX timing!");
    //   return false;
    // }

    // uint16_t signalThreshold = -1;
    // if (_VL53L4CX->VL53L4CX_GetSignalThreshold(&signalThreshold) !=
    //     VL53L4CX_ERROR_NONE) {
    //   WS_DEBUG_PRINTLN("Failed to get VL53L4CX signal threshold!");
    // } else {
    //   WS_DEBUG_PRINT("VL53L4CX old signal threshold: ");
    //   WS_DEBUG_PRINTLN(signalThreshold);
    // }
    // if (_VL53L4CX->VL53L4CX_SetSignalThreshold(50) != VL53L4CX_ERROR_NONE) {
    //   WS_DEBUG_PRINTLN("Failed to set VL53L4CX signal threshold!");
    // } else {
    //   WS_DEBUG_PRINT("VL53L4CX new signal threshold: ");
    //   WS_DEBUG_PRINTLN(8);
    // }

    // uint16_t sigmaThreshold = -1;
    // if (_VL53L4CX->VL53L4CX_GetSigmaThreshold(&sigmaThreshold) !=
    //     VL53L4CX_ERROR_NONE) {
    //   WS_DEBUG_PRINTLN("Failed to get VL53L4CX sigma threshold!");
    // } else {
    //   WS_DEBUG_PRINT("VL53L4CX old sigma threshold: ");
    //   WS_DEBUG_PRINTLN(sigmaThreshold);
    // }
    // if (_VL53L4CX->VL53L4CX_SetSigmaThreshold(100) != VL53L4CX_ERROR_NONE) {
    //   WS_DEBUG_PRINTLN("Failed to set VL53L4CX sigma threshold!");
    // } else {
    //   WS_DEBUG_PRINT("VL53L4CX new sigma threshold: ");
    //   WS_DEBUG_PRINTLN(100);
    // }

    if (_VL53L4CX->VL53L4CX_StartMeasurement() != VL53L4CX_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to start VL53L4CX ranging!");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL53L4CX's current proximity for first object if found.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    return getProximity(proximityEvent, 0);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL53L4CX's current proximity for second object if
     found.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *proximityEvent) {
    return getProximity(proximityEvent, 1);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL53L4CX's current proximity (first or second object).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @param    index
                Index of the proximity to get (0 or 1).
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getProximity(sensors_event_t *proximityEvent, int index = 0) {
    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0;
    int j = 0;
    int status;
    // Start fresh reading, seemed to be accepting stale value
    status = _VL53L4CX->VL53L4CX_ClearInterruptAndStartMeasurement();
    WS_DEBUG_PRINT("Waiting for VL53L4CX data ready...");
    delay(250);

    for (uint8_t i = 0; (status = _VL53L4CX->VL53L4CX_GetMeasurementDataReady(
                             &NewDataReady)) &&
                        !NewDataReady && i < 3;
         i++) {
      delay(300);
      WS_DEBUG_PRINT(" .");
    }
    WS_DEBUG_PRINTLN("");
    if ((status == VL53L4CX_ERROR_NONE) && (NewDataReady != 0)) {
      status = _VL53L4CX->VL53L4CX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

      for (j = 0; j < no_of_object_found; j++) {
        if (pMultiRangingData->RangeData[j].RangeStatus ==
                VL53L4CX_RANGESTATUS_RANGE_VALID ||
            pMultiRangingData->RangeData[j].RangeStatus ==
                VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE) {
          int16_t mm = pMultiRangingData->RangeData[j].RangeMilliMeter;
          if (j == index) {
            proximityEvent->data[0] = (float)mm;
            return true;
          }
        }
      }
      // TODO: Once I2C sensors fire all data points during a single call, we
      // can return both distances and other metrics like the std deviation
    } else {
      WS_DEBUG_PRINTLN("VL53L4CX Error: " +
                       String(getVL53L4CXStatusDescription(status)));
    }
    return false;
  }

protected:
  VL53L4CX *_VL53L4CX; ///< Pointer to VL53L4CX temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L4CX