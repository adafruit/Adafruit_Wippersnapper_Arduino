/*!
 * @file drvVl53l0x.h
 *
 * Device driver for the VL53L0X ToF sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2022 afp316 https://github.com/afp316
 * Modified (c) by Tyeth Gundry https://github.com/tyeth
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_VL53L0X_H
#define DRV_VL53L0X_H

#include "drvBase.h"
#include <Adafruit_VL53L0X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L0X sensor.
*/
/**************************************************************************/
class drvVl53l0x : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L0X sensor.
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
  drvVl53l0x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L0X sensor.
  */
  /*******************************************************************************/
  ~drvVl53l0x() {
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
    bool isInit =
        _vl53l0x->begin((uint8_t)_address, false, _i2c,
                        Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
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
    u_int16_t proximityMM = _vl53l0x->readRange();
    if (proximityMM == 0xffff) {
      proximityEvent->data[0] = NAN;
    } else {
      proximityEvent->data[0] = proximityMM;
    }
    return true;
  }

protected:
  Adafruit_VL53L0X *_vl53l0x; ///< Pointer to VL53L0X temperature sensor object
};

#endif // drvVl53l0x