/*!
 * @file WipperSnapper_I2C_Driver_VL53L1X.h
 *
 * Device driver for the VL53L1X ToF sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2022 afp316 https://github.com/afp316
 * Modified  Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_VL53L1X_H
#define WipperSnapper_I2C_Driver_VL53L1X_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_VL53L1X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L1X sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VL53L1X : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L1X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VL53L1X(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L1X sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VL53L1X() {
    // Called when a VL53L1X component is deleted.
    delete _VL53L1X;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL53L1X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _VL53L1X = new Adafruit_VL53L1X();
    if (_VL53L1X->begin((uint8_t)_sensorAddress, _i2c, false)) {
      _VL53L1X->startRanging();
      _VL53L1X->setTimingBudget(500); // distance mode is long(2) by default
      return true;
    }
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL53L1X's current proximity.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    if (!_VL53L1X->dataReady()) {
      return false;
    }
    int16_t proximityMM = _VL53L1X->distance();
    if (proximityMM <= 0 || proximityMM > 4000) {
      WS_DEBUG_PRINT("VL53L1X: Invalid proximity value:");
      WS_DEBUG_PRINTLN(proximityMM);
      WS_DEBUG_PRINT("VL53L1X status: ");
      WS_DEBUG_PRINTLN(_VL53L1X->vl_status);
      proximityEvent->data[0] = NAN;
    } else {
      proximityEvent->data[0] = proximityMM;
    }
    _VL53L1X->clearInterrupt();
    return true;
  }

protected:
  Adafruit_VL53L1X *_VL53L1X; ///< Pointer to VL53L1X temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L1X