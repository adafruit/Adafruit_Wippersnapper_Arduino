/*!
 * @file WipperSnapper_I2C_Driver_VL53L0X.h
 *
 * Device driver for the VL53L0X ToF sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_VL53L0X_H
#define WipperSnapper_I2C_Driver_VL53L0X_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_VL53L0X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L0X sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VL53L0X : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L0X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VL53L0X(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L0X sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VL53L0X() {
    // Called when a VL53L0X component is deleted.
    delete _vl53l0x;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL53L0X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _vl53l0x = new Adafruit_VL53L0X();
    bool isInit = _vl53l0x->begin((uint8_t)_sensorAddress, _i2c);
    return isInit;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL53L0X's current proximity.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->proximity = _vl53l0x->readRange();
    return true;
  }

protected:
  Adafruit_VL53L0X *_vl53l0x; ///< Pointer to VL53L0X temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L0X