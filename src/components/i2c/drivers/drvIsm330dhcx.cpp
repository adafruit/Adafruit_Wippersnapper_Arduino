/*!
 * @file drvIsm330dhcx.cpp
 *
 * Driver wrapper for the Adafruit ISM330DHCX (LSM6DSOX core) 6-DoF IMU.
 */

#include "drvIsm330dhcx.h"

#include <math.h>

drvIsm330dhcx::drvIsm330dhcx(TwoWire *i2c, uint16_t sensorAddress,
                             uint32_t mux_channel, const char *driver_name)
    : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

drvIsm330dhcx::~drvIsm330dhcx() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }
}

bool drvIsm330dhcx::begin() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }

  _imu = new Adafruit_ISM330DHCX();
  if (!_imu) {
    return false;
  }

  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvIsm330dhcx] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  if (!_imu->begin_I2C(addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvIsm330dhcx] Failed to initialise sensor");
    delete _imu;
    _imu = nullptr;
    return false;
  }

  _imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  _imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
  _imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  _imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
  _imu->highPassFilter(true, LSM6DS_HPF_ODR_DIV_100);
  _imu->configInt1(false, false, false, false, true);
  _imu->configInt2(false, false, false);
  _imu->enablePedometer(true);
  _imu->enableWakeup(true);

  WS_DEBUG_PRINTLN("[drvIsm330dhcx] Sensor initialised successfully");
  return true;
}

bool drvIsm330dhcx::readAllEvents(sensors_event_t *accel, sensors_event_t *gyro,
                                  sensors_event_t *temp, boolean *shake, uint16_t *steps) {
  if (!_imu) {
    return false;
  }
  if (_lastPoll > 0 && _internalPollPeriod > 0 && millis() - _lastPoll < _internalPollPeriod) {
    // Limit polling to every 200ms, return cache, but not first time
    return true;
  }
  _lastPoll = millis(); // TODO: set in fastTicks, if used, instead
  if (_imu->shake()) {
    WS_DEBUG_PRINTLN("[drvIsm330dhcx] Shake detected!");
    *shake = true;
  }
  uint16_t step_change = _imu->readPedometer();
  if (step_change > 0) {
    WS_DEBUG_PRINT("[drvIsm330dhcx] Steps detected: ")
    WS_DEBUG_PRINTLN(step_change);
    steps = steps + step_change;
    _imu->resetPedometer();
  }
  return _imu->getEvent(accel, gyro, temp);
}

bool drvIsm330dhcx::readAllEvents() {
  return readAllEvents(&_lastAccelEvent, &_lastGyroEvent, &_lastTempEvent,
                       &_last_shake, &_last_steps);
}

bool drvIsm330dhcx::computeAccelMagnitude(float &magnitude) {
  if (!readAllEvents()) {
    return false;
  }
  magnitude = sqrtf(_lastAccelEvent.acceleration.x * _lastAccelEvent.acceleration.x +
                    _lastAccelEvent.acceleration.y * _lastAccelEvent.acceleration.y +
                    _lastAccelEvent.acceleration.z * _lastAccelEvent.acceleration.z);
  return true;
}

bool drvIsm330dhcx::getEventRaw(sensors_event_t *rawEvent) {
  if (!_imu) {
    return false;
  }

  if (!readAllEvents()) {
    return false;
  }
  // Return step count as raw event, counter cleared at read
  rawEvent->data[0] = (float)_last_steps;
  _last_steps = 0;
  return true;
  

  ////MAGNITUDE ONLY - DISABLED FOR NOW
  // WS_DEBUG_PRINTLN("[drvIsm330dhcx] Getting raw magnitude event...");
  // float magnitude = 0.0f;
  // if (!computeAccelMagnitude(magnitude)) {
  //   return false;
  // }
  // rawEvent->data[0] = magnitude;
  // WS_DEBUG_PRINT("[drvIsm330dhcx] Raw magnitude: ");
  // WS_DEBUG_PRINTLN(magnitude);
  // return true;
}

bool drvIsm330dhcx::getEventBoolean(sensors_event_t *booleanEvent) {
  if (!_imu) {
    return false;
  }
  if (!readAllEvents()) {
    return false;
  }
  WS_DEBUG_PRINT("[drvIsm330dhcx] Checking for shake/tap/threshold event...");
  WS_DEBUG_PRINTLN(_last_shake ? " DETECTED!" : " none.");
  booleanEvent->data[0] = _last_shake ? 1.0f : 0.0f;
  if (_last_shake) {
    WS_DEBUG_PRINTLN("[drvIsm330dhcx] *** Threshold event detected ***");
  }
  _last_shake = false;
  return true;
}

bool drvIsm330dhcx::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN("[drvIsm330dhcx] Getting accelerometer event...");
  bool result = readAllEvents()
                    ? memcpy(accelEvent, &_lastAccelEvent, sizeof(sensors_event_t)),
                    true
                    : false;
  return result;
}

bool drvIsm330dhcx::getEventGyroscope(sensors_event_t *gyroEvent) {
  if (!_imu) {
    return false;
  }
  WS_DEBUG_PRINTLN("[drvIsm330dhcx] Getting gyroscope event...");
  bool result = readAllEvents()
                    ? memcpy(gyroEvent, &_lastGyroEvent, sizeof(sensors_event_t)),
                    true
                    : false;
  return result;
}

void drvIsm330dhcx::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
}
