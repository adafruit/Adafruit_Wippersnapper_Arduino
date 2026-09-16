/*!
 * @file drvVl53l4cd.h
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
#ifndef DRV_VL53L4CD
#define DRV_VL53L4CD

#include "drvBase.h"
#include <vl53l4cd_class.h>

#define VL53L4CD_TICK_MS 50        ///< Poll for a finished range every 50ms
#define VL53L4CD_READ_LEAD_MS 1000 ///< Start ranging 1s before a read is due

/*!
    @brief  Class that provides a driver interface for a VL53L4CD sensor.
*/
class drvVl53l4cd : public drvBase {
public:
  /*!
      @brief    Constructor for a VL53L4CD sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvVl53l4cd(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // A range takes up to the 200ms timing budget, so it is collected in the
    // background by fastTick() during the lead window before each read is due
    // rather than blocking the read pass.
    _fast_tick_ms = VL53L4CD_TICK_MS;
    _tick_lead_ms = VL53L4CD_READ_LEAD_MS;
  }

  /*!
      @brief    Destructor for an VL53L4CD sensor.
  */
  ~drvVl53l4cd() {
    // Called when a VL53L4CD component is deleted.
    delete _VL53L4CD;
  }

  /*!
      @brief    Initializes the VL53L4CD sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() {
    _VL53L4CD = new VL53L4CD(_i2c, -1);

    if (_VL53L4CD->InitSensor((uint8_t)_address) != VL53L4CD_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to initialize VL53L4CD sensor!");
      return false;
    }
    // Program the highest possible TimingBudget, no interval time
    if (_VL53L4CD->VL53L4CD_SetRangeTiming(200, 0) != VL53L4CD_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to set VL53L4CD timing!");
      return false;
    }

    if (uint16_t signalThreshold;
        _VL53L4CD->VL53L4CD_GetSignalThreshold(&signalThreshold) ==
        VL53L4CD_ERROR_NONE) {
      // WS_DEBUG_PRINT("VL53L4CD old signal threshold: ");
      // WS_DEBUG_PRINTLN(signalThreshold);
      // WS_DEBUG_PRINTLN("Setting VL53L4CD signal threshold to 50");
      if (_VL53L4CD->VL53L4CD_SetSignalThreshold(50) != VL53L4CD_ERROR_NONE) {
        // WS_DEBUG_PRINTLN("Failed to set new VL53L4CD signal threshold!");
      }
    } else {
      // WS_DEBUG_PRINTLN("Failed to get VL53L4CD signal threshold!");
    }

    if (uint16_t sigmaThreshold; _VL53L4CD->VL53L4CD_GetSigmaThreshold(
                                     &sigmaThreshold) == VL53L4CD_ERROR_NONE) {
      // WS_DEBUG_PRINT("VL53L4CD old sigma threshold: ");
      // WS_DEBUG_PRINTLN(sigmaThreshold);
      // WS_DEBUG_PRINTLN("Setting VL53L4CD sigma threshold to 100");
      if (_VL53L4CD->VL53L4CD_SetSigmaThreshold(100) != VL53L4CD_ERROR_NONE) {
        // WS_DEBUG_PRINTLN("Failed to set VL53L4CD sigma threshold!");
      }
    } else {
      // WS_DEBUG_PRINTLN("Failed to get VL53L4CD sigma threshold!");
    }

    if (_VL53L4CD->VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
      // WS_DEBUG_PRINTLN("Failed to start VL53L4CD ranging!");
      return false;
    }
    return true;
  }

  /*!
      @brief    Background ranging step, called every VL53L4CD_TICK_MS while a
                read is pending. On resuming after idle the pending interrupt
                is cleared so a stale result completed long ago is discarded
                ("seemed to be accepting stale value"). Otherwise, if a range
                has completed, it is read, the interrupt cleared to start the
                next one, and a valid result filed with NewSample().
  */
  void fastTick() override {
    ulong now = millis();
    bool resumed = now - _last_tick_ms > 2 * VL53L4CD_TICK_MS;
    _last_tick_ms = now;
    if (resumed) {
      _VL53L4CD->VL53L4CD_ClearInterrupt();
      return;
    }

    uint8_t ready = 0;
    if (_VL53L4CD->VL53L4CD_CheckForDataReady(&ready) != VL53L4CD_ERROR_NONE ||
        !ready)
      return;

    VL53L4CD_Result_t results = {0};
    uint8_t status = _VL53L4CD->VL53L4CD_GetResult(&results);
    // (Mandatory) Clear HW interrupt to restart measurements
    _VL53L4CD->VL53L4CD_ClearInterrupt();
    // RangeStatus = 0 means valid data; otherwise keep waiting for a good one
    if (status != VL53L4CD_ERROR_NONE || results.range_status != 0)
      return;
    // NOTE: results also carries sigma_mm (std deviation) should we want it.
    _distance_mm = results.distance_mm;
    NewSample();
  }

  /*!
      @brief    Gets the VL53L4CD's current proximity, from the most recent
                valid range collected by fastTick().
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  bool getEventProximity(sensors_event_t *proximityEvent) {
    if (!AttemptRead())
      return false;
    proximityEvent->data[0] = (float)_distance_mm;
    return true;
  }

protected:
  VL53L4CD *_VL53L4CD;       ///< Pointer to VL53L4CD sensor object
  uint16_t _distance_mm = 0; ///< Last valid range, in mm
  ulong _last_tick_ms = 0;   ///< millis() of the last fastTick() call
};

#endif // drvVl53l4cd