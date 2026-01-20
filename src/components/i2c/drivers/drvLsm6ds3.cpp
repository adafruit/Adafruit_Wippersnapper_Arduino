/*!
 * @file drvLsm6ds3.cpp
 *
 * Driver wrapper for the Adafruit LSM6DS3 6-DoF IMU.
 */

#include "drvLsm6ds3.h"

/******************************************************************************/
/*! @brief Destructor */
/******************************************************************************/
drvLsm6ds3::~drvLsm6ds3() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }
}

/******************************************************************************/
bool drvLsm6ds3::begin() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }

  _imu = new Adafruit_LSM6DS3();
  if (!_imu) {
    return false;
  }

  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvLsm6ds3] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  if (!_imu->begin_I2C(addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm6ds3] Failed to initialise sensor");
    delete _imu;
    _imu = nullptr;
    return false;
  }

  _imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  _imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
  _imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  _imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
  _imu->configInt1(false, false, false, false, true);
  _imu->configInt2(false, false, false);
  // _imu->enablePedometer(true);
  _imu->enableWakeup(true);

  WS_DEBUG_PRINTLN("[drvLsm6ds3] Sensor initialised successfully");
  return true;
}

