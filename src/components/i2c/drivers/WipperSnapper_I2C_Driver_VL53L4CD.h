#ifndef WipperSnapper_I2C_Driver_VL53L4CD_H
#define WipperSnapper_I2C_Driver_VL53L4CD_H

#include "WipperSnapper_I2C_Driver.h"
#include "vl53l4cd_class.h"

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
    delete _vl53l4cd;
  }

  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->proximity = _vl53l4cd->VL53L4CD_StartRanging();
    return true;
  }

protected:
  VL53L4CD *_vl53l4cd; ///< Pointer to VL53L4CD ToF sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L4CD