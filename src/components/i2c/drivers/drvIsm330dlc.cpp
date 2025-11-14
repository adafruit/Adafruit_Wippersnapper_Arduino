/*!
 * @file drvIsm330dlc.cpp
 *
 * Driver wrapper for the Adafruit ISM330DLC (LSM6DSL core) 6-DoF IMU.
 */

#include "drvIsm330dlc.h"

#include <math.h>

drvIsm330dlc::drvIsm330dlc(TwoWire *i2c, uint16_t sensorAddress,
                           uint32_t mux_channel, const char *driver_name)
    : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

drvIsm330dlc::~drvIsm330dlc() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }
}

bool drvIsm330dlc::begin() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }

  _imu = new Adafruit_LSM6DSL();
  if (!_imu) {
    return false;
  }

  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvIsm330dlc] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  if (!_imu->begin_I2C(addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvIsm330dlc] Failed to initialise sensor");
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
  _imu->enableWakeup(true);

  WS_DEBUG_PRINTLN("[drvIsm330dlc] Sensor initialised successfully");
  return true;
}

bool drvIsm330dlc::readAllEvents(sensors_event_t *accel,
                                sensors_event_t *gyro,
                                sensors_event_t *temp) {
  if (!_imu) {
    return false;
  }
  return _imu->getEvent(accel, gyro, temp);
}

bool drvIsm330dlc::computeAccelMagnitude(float &magnitude) {
  sensors_event_t accel, gyro, temp;
  if (!readAllEvents(&accel, &gyro, &temp)) {
    return false;
  }
  magnitude = sqrtf(accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.y * accel.acceleration.y +
                    accel.acceleration.z * accel.acceleration.z);
  return true;
}

bool drvIsm330dlc::getEventRaw(sensors_event_t *rawEvent) {
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN("[drvIsm330dlc] Getting raw magnitude event...");
  float magnitude = 0.0f;
  if (!computeAccelMagnitude(magnitude)) {
    return false;
  }
  rawEvent->data[0] = magnitude;
  WS_DEBUG_PRINT("[drvIsm330dlc] Raw magnitude: ");
  WS_DEBUG_PRINTLN(magnitude);
  return true;
}

bool drvIsm330dlc::getEventBoolean(sensors_event_t *booleanEvent) {
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN("[drvIsm330dlc] Checking for tap/threshold event...");
  bool tap = _imu->shake();
  booleanEvent->data[0] = tap ? 1.0f : 0.0f;
  if (tap) {
    WS_DEBUG_PRINTLN("[drvIsm330dlc] Threshold event detected");
  }
  return true;
}

bool drvIsm330dlc::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN("[drvIsm330dlc] Getting accelerometer event...");
  sensors_event_t gyro, temp;
  return readAllEvents(accelEvent, &gyro, &temp);
}

bool drvIsm330dlc::getEventGyroscope(sensors_event_t *gyroEvent) {
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN("[drvIsm330dlc] Getting gyroscope event...");
  sensors_event_t accel, temp;
  return readAllEvents(&accel, gyroEvent, &temp);
}

void drvIsm330dlc::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
}
