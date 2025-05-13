/*!
 * @file drvVl53l4cx.h
 *
 * Device driver for the VL53L4CX ToF sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2024 Tyeth Gundry for Adafruit Industries
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_VL53L4CX_H
#define DRV_VL53L4CX_H

#include "drvBase.h"
#include <vl53l4cx_class.h>
#include <vl53l4cx_def.h>

#define VL53_SHUTDOWN_PIN -1         ///< Shutdown pin for VL53L4CX sensor
#define VL53_READING_DELAY 250       ///< Delay for reading data attempts
#define VL53_TIMING_BUDGET_NS 200000 ///< Timing budget for VL53L4CX sensor

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L4CX sensor.
*/
/**************************************************************************/
class drvVl53l4cx : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L4CX sensor.
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
  drvVl53l4cx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L4CX sensor.
  */
  /*******************************************************************************/
  ~drvVl53l4cx() {
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
    _VL53L4CX = new VL53L4CX(_i2c, VL53_SHUTDOWN_PIN);

    if (_VL53L4CX->InitSensor((uint8_t)_address) != VL53L4CX_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to initialize VL53L4CX sensor!");
      return false;
    }

    if (_VL53L4CX->VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG) !=
        VL53L4CX_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to set VL53L4CX distance mode to long!");
      return false;
    }

    // Set 200ms measurement time, the possible TimingBudget is 8-200ms
    if (_VL53L4CX->VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(
            VL53_TIMING_BUDGET_NS) != VL53L4CX_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to set VL53L4CX timing budget!");
      return false;
    }

    if (_VL53L4CX->VL53L4CX_StartMeasurement() != VL53L4CX_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to start VL53L4CX ranging!");
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
      @param    whichObject
                Index of the proximity object to get (0, or 1 for second
     object).
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getProximity(sensors_event_t *proximityEvent, int whichObject = 0) {
    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int status;

    // Start fresh reading, seemed to be accepting stale value
    status = _VL53L4CX->VL53L4CX_ClearInterruptAndStartMeasurement();
    if (status != VL53L4CX_ERROR_NONE) {
      // WS_DEBUG_PRINT("VL53L4CX Error clearing interrupt and starting
      // measurement: "); WS_DEBUG_PRINTLN(status);
      return false;
    }

    // Wait for data read period then update data ready status
    // WS_DEBUG_PRINT("Waiting for VL53L4CX data ready...");
    delay(VL53_READING_DELAY);
    status = _VL53L4CX->VL53L4CX_GetMeasurementDataReady(&NewDataReady);

    if ((status != VL53L4CX_ERROR_NONE) || (NewDataReady == 0)) {
      // error or no data ready
      // WS_DEBUG_PRINT("VL53L4CX Error checking for data ready: ");
      // WS_DEBUG_PRINTLN(status);
      return false;
    }

    // get data - still to verify which of one or two objects found
    status = _VL53L4CX->VL53L4CX_GetMultiRangingData(pMultiRangingData);
    if (status != VL53L4CX_ERROR_NONE) {
      // WS_DEBUG_PRINT("VL53L4CX Error getting multi ranging data: ");
      // WS_DEBUG_PRINTLN(status);
      return false;
    }

    // whichObject: 0-based index, return NaN(Object not found) if too few found
    if (pMultiRangingData->NumberOfObjectsFound - 1 < whichObject) {
      // WS_DEBUG_PRINT("Object not found at index #");
      // WS_DEBUG_PRINT(whichObject + 1); // human readable 1-based index
      // WS_DEBUG_PRINTLN(", returning NaN");
      proximityEvent->data[0] = NAN;
      return true;
    }

    // RESULT: take the first or second detected object from ranging data,
    // if valid then set event data in proximityEvent or return false
    return updateDataPointIfValid(pMultiRangingData->RangeData[whichObject],
                                  proximityEvent);
  }

  /*******************************************************************************/
  /*!
      @brief    Checks the VL53L4CX's proximity result and sets event value.
      @param    rangingData
                The ranging data to check.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool updateDataPointIfValid(VL53L4CX_TargetRangeData_t rangingData,
                              sensors_event_t *proximityEvent) {
    if (rangingData.RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID) {
      int16_t mm = rangingData.RangeMilliMeter;
      proximityEvent->data[0] = (float)mm;
      return true;
    }
    return false;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_PROXIMITY;
    _default_sensor_types[1] = wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  }

protected:
  VL53L4CX *_VL53L4CX; ///< Pointer to VL53L4CX temperature sensor object
};

#endif // drvVl53l4cx