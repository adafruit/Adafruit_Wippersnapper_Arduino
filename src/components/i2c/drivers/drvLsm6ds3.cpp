/*!
 * @file drvLsm6ds3.cpp
 *
 * Driver wrapper for the Adafruit LSM6DS3 and LSM6DS3TR-C 6-DoF IMUs.
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

  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvLsm6ds3] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  // Pick the library class from WHO_AM_I (a read only, nothing is written
  // before the part is identified): the original LSM6DS3 reports 0x69, the
  // LSM6DS3TR-C (current Adafruit LSM6DS3 boards) reports 0x6A.
  Adafruit_I2CDevice dev(addr, _i2c);
  if (!dev.begin()) {
    WS_DEBUG_PRINTLN("[drvLsm6ds3] No device at address");
    return false;
  }
  Adafruit_BusIO_Register whoami(&dev, LSM6DS_WHOAMI);
  uint8_t chip_id = whoami.read();
  if (chip_id == LSM6DS3TRC_CHIP_ID) {
    _imu = new Adafruit_LSM6DS3TRC();
  } else if (chip_id == LSM6DS3_CHIP_ID) {
    _imu = new Adafruit_LSM6DS3();
  } else {
    WS_DEBUG_PRINT("[drvLsm6ds3] Not an LSM6DS3/LSM6DS3TR-C, WHO_AM_I 0x");
    WS_DEBUG_PRINTHEX(chip_id);
    WS_DEBUG_PRINTLN("");
    return false;
  }
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN(chip_id == LSM6DS3TRC_CHIP_ID ? "[drvLsm6ds3] LSM6DS3TR-C"
                                                 : "[drvLsm6ds3] LSM6DS3");

  if (!_imu->begin_I2C(addr, _i2c, getLsmSensorID())) {
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
