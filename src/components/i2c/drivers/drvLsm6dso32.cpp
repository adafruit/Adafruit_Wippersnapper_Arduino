/*!
 * @file drvLsm6dso32.cpp
 *
 * Driver wrapper for the Adafruit LSM6DSO32 6-DoF IMU.
 */

#include "drvLsm6dso32.h"

#include <math.h>

/******************************************************************************/
drvLsm6dso32::~drvLsm6dso32() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }
}

/******************************************************************************/
bool drvLsm6dso32::begin() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }

  _imu = new Adafruit_LSM6DSO32();
  if (!_imu) {
    return false;
  }

  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvLsm6dso32] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  if (!_imu->begin_I2C(addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm6dso32] Failed to initialise sensor");
    delete _imu;
    _imu = nullptr;
    return false;
  }

  _imu->setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  _imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
  _imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  _imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
  _imu->configInt1(false, false, false, false, true);
  _imu->configInt2(false, false, false);
  _imu->enableWakeup(true);

  WS_DEBUG_PRINTLN("[drvLsm6dso32] Sensor initialised successfully");
  return true;
}

bool drvLsm6dso32::readAllEvents(sensors_event_t *accel,
                                 sensors_event_t *gyro,
                                 sensors_event_t *temp) {
  if (!_imu) {
    return false;
  }
  return _imu->getEvent(accel, gyro, temp);
}

bool drvLsm6dso32::computeAccelMagnitude(float &magnitude) {
  sensors_event_t accel, gyro, temp;
  if (!readAllEvents(&accel, &gyro, &temp)) {
    return false;
  }
  magnitude = sqrtf(accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.y * accel.acceleration.y +
                    accel.acceleration.z * accel.acceleration.z);
  return true;
}

bool drvLsm6dso32::getEventRaw(sensors_event_t *rawEvent) {
  if (!_imu) {
    return false;
  }
  float magnitude = 0.0f;
  if (!computeAccelMagnitude(magnitude)) {
    return false;
  }
  rawEvent->data[0] = magnitude;
  return true;
}

bool drvLsm6dso32::getEventBoolean(sensors_event_t *booleanEvent) {
  if (!_imu) {
    return false;
  }
  bool shake = _imu->shake();
  booleanEvent->data[0] = shake ? 1.0f : 0.0f;
  return true;
}

bool drvLsm6dso32::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!_imu) {
    return false;
  }
  sensors_event_t gyro, temp;
  return readAllEvents(accelEvent, &gyro, &temp);
}

bool drvLsm6dso32::getEventGyroscope(sensors_event_t *gyroEvent) {
  if (!_imu) {
    return false;
  }
  sensors_event_t accel, temp;
  return readAllEvents(&accel, gyroEvent, &temp);
}

void drvLsm6dso32::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
}
