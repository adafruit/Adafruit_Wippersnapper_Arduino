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

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL53L4CX sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _VL53L4CX = new VL53L4CX(_i2c, -1);

    if (_VL53L4CX->InitSensor((uint8_t)_sensorAddress) != VL53L4CX_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to initialize VL53L4CX sensor!");
      return false;
    }
    
    // Set 1 second measurement time, the highest possible TimingBudget is 10seconds
    if (_VL53L4CX->VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(1000000) != VL53L4CX_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to set VL53L4CX timing budget!");
      return false;
    }

    if (_VL53L4CX->VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG) != VL53L4CX_ERROR_NONE) {
      WS_DEBUG_PRINTLN("Failed to set VL53L4CX distance mode to long!");
    }

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
    int currentObject = 0;
    int status;
    // Start fresh reading, seemed to be accepting stale value
    status = _VL53L4CX->VL53L4CX_ClearInterruptAndStartMeasurement();
    WS_DEBUG_PRINT("Waiting for VL53L4CX data ready...");
    delay(250);

    for (uint8_t retries = 0;
         (status = _VL53L4CX->VL53L4CX_GetMeasurementDataReady(&NewDataReady)) &&
         !NewDataReady && retries < 3;
         retries++) {
      delay(300);
      WS_DEBUG_PRINT(" .");
    }
    WS_DEBUG_PRINTLN("");
    if ((status == VL53L4CX_ERROR_NONE) && (NewDataReady != 0)) {
      status = _VL53L4CX->VL53L4CX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

      for (currentObject = 0; currentObject < no_of_object_found;
           currentObject++) {
        if (pMultiRangingData->RangeData[currentObject].RangeStatus ==
                VL53L4CX_RANGESTATUS_RANGE_VALID ||
            pMultiRangingData->RangeData[currentObject].RangeStatus ==
                VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE) {
          int16_t mm =
              pMultiRangingData->RangeData[currentObject].RangeMilliMeter;
          if (currentObject == index) {
            proximityEvent->data[0] = (float)mm;
            return true;
          }
        }
      }
    } else {
      WS_DEBUG_PRINT("VL53L4CX Error: ");
      WS_DEBUG_PRINTLN(status);
    }
    return false;
  }

protected:
  VL53L4CX *_VL53L4CX; ///< Pointer to VL53L4CX temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L4CX