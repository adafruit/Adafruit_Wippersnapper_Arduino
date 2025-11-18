/*!
 * @file drvBaseAccelLsm6.cpp
 */

#include "drvBaseAccelLsm6.h"

#include <math.h>

drvBaseAccelLsm6::drvBaseAccelLsm6(TwoWire *i2c, uint16_t sensorAddress,
                                   uint32_t mux_channel,
                                   const char *driver_name)
    : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

drvBaseAccelLsm6::~drvBaseAccelLsm6() {}

void drvBaseAccelLsm6::setInternalPollingInterval(uint32_t interval_ms) {
  _internalPollPeriod = interval_ms;
}

bool drvBaseAccelLsm6::readAllEvents() {
  Adafruit_LSM6DS *imu = getLSM6Sensor();
  if (!imu) {
    return false;
  }

  uint32_t now = millis();
  if (_has_last_events && _internalPollPeriod > 0 &&
      (now - _lastPoll) < _internalPollPeriod) {
    // too soon reuse cached data, except first run or interval=0
    return true; 
  }

  _lastPoll = now;

  if (imu->shake()) {
    WS_DEBUG_PRINT("[");
    WS_DEBUG_PRINT(_name);
    WS_DEBUG_PRINTLN("] Shake detected!");
    _last_shake = true;
  }

//   uint16_t step_change = imu->readPedometer();
//   if (step_change > 0) {
//     WS_DEBUG_PRINT("[");
//     WS_DEBUG_PRINT(_name);
//     WS_DEBUG_PRINT("] Steps detected !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n!: ");
//     WS_DEBUG_PRINTLN(step_change);
//     _last_steps += step_change;
//     imu->resetPedometer();
//   }

  bool success = imu->getEvent(&_lastAccelEvent, &_lastGyroEvent, &_lastTempEvent);
  _has_last_events = success;
  return success;
}

bool drvBaseAccelLsm6::computeAccelMagnitude(float *magnitude) {
  if (!readAllEvents()) {
    return false;
  }

  *magnitude = sqrtf(_lastAccelEvent.acceleration.x *
                        _lastAccelEvent.acceleration.x +
                    _lastAccelEvent.acceleration.y *
                        _lastAccelEvent.acceleration.y +
                    _lastAccelEvent.acceleration.z *
                        _lastAccelEvent.acceleration.z);
  return true;
}

bool drvBaseAccelLsm6::getEventRaw(sensors_event_t *rawEvent) {
  if (!readAllEvents()) {
    return false;
  }
  return computeAccelMagnitude(&(rawEvent->data[0]));
}

bool drvBaseAccelLsm6::getEventAmbientTemp(sensors_event_t *temperatureEvent) {
  if (!readAllEvents()) {
    return false;
  }

  *temperatureEvent = _lastTempEvent;
  return true;
}

// NO Shake/tap/wakeup for now, using pedometer steps instead on INT1
bool drvBaseAccelLsm6::getEventBoolean(sensors_event_t *booleanEvent) {
  if (!readAllEvents()) {
    return false;
  }

  booleanEvent->data[0] = _last_shake ? 1.0f : 0.0f;
  if (_last_shake) {
    WS_DEBUG_PRINT("[");
    WS_DEBUG_PRINT(_name);
    WS_DEBUG_PRINTLN("] *** Threshold event detected ***");
    _last_shake = false;
  }
  return true;
}

bool drvBaseAccelLsm6::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!readAllEvents()) {
    return false;
  }

  *accelEvent = _lastAccelEvent;
  return true;
}

bool drvBaseAccelLsm6::getEventGyroscope(sensors_event_t *gyroEvent) {
  if (!readAllEvents()) {
    return false;
  }

  *gyroEvent = _lastGyroEvent;
  return true;
}

void drvBaseAccelLsm6::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
}
