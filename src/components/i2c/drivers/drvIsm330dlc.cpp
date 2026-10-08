/*!
 * @file drvIsm330dlc.cpp
 *
 * Driver wrapper for the Adafruit ISM330DLC (LSM6DSL core) 6-DoF IMU.
 */

#include "drvIsm330dlc.h"

/******************************************************************************/
/*!
    @brief    Constructor for an ISM330DLC sensor.
    @param    i2c
              The I2C interface.
    @param    sensorAddress
              7-bit device address.
    @param    mux_channel
              The I2C multiplexer channel.
    @param    driver_name
              The name of the driver.
*/
/******************************************************************************/
drvIsm330dlc::drvIsm330dlc(TwoWire *i2c, uint16_t sensorAddress,
                           uint32_t mux_channel, const char *driver_name)
    : drvBaseAccelLsm6(i2c, sensorAddress, mux_channel, driver_name) {}

/******************************************************************************/
/*!
    @brief    Destructor for an ISM330DLC sensor.
*/
/******************************************************************************/
drvIsm330dlc::~drvIsm330dlc() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }
}

/******************************************************************************/
/*!
    @brief    Initializes the ISM330DLC sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/******************************************************************************/
bool drvIsm330dlc::begin() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }

  _imu = new Adafruit_LSM6DSL();
  if (!_imu) {
    return false;
  }

  // NOTE: WHO_AM_I 0x6A is shared by ISM330DLC, LSM6DSL and LSM6DS3TR-C, so
  // autoconfig will name any of them "ism330dlc" (register compatible).
  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvIsm330dlc] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  if (!_imu->begin_I2C(addr, _i2c, getLsmSensorID())) {
    WS_DEBUG_PRINTLN("[drvIsm330dlc] Failed to initialise sensor");
    delete _imu;
    _imu = nullptr;
    return false;
  }

  _imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  _imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
  _imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  _imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
  // _imu->highPassFilter(true, LSM6DS_HPF_ODR_DIV_100);
  _imu->configInt1(false, false, false, false, true);
  _imu->configInt2(false, false, false);
  // _imu->enablePedometer(true);
  _imu->enableWakeup(true);

  WS_DEBUG_PRINTLN("[drvIsm330dlc] Sensor initialised successfully");
  return true;
}
