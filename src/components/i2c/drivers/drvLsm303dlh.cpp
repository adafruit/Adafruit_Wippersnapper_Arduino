/*!
 * @file drvLsm303dlh.cpp
 *
 * Driver wrapper for the legacy Adafruit LSM303DLH combo sensor.
 */

#include "drvLsm303dlh.h"

#include <math.h>

namespace {
constexpr uint8_t kLsm303dlhAccelDefaultAddr = LSM303_ADDRESS_ACCEL;
constexpr uint8_t kLsm303dlhMagDefaultAddr = 0x1E;
}

/******************************************************************************/
/*! @brief Destructor */
/******************************************************************************/
drvLsm303dlh::~drvLsm303dlh() { teardown(); }

/******************************************************************************/
void drvLsm303dlh::teardown() {
  if (_accel) {
    delete _accel;
    _accel = nullptr;
  }
  if (_mag) {
    delete _mag;
    _mag = nullptr;
  }
}

/******************************************************************************/
bool drvLsm303dlh::begin() {
  teardown();

  _accel = new Adafruit_LSM303_Accel_Unified();
  _mag = new Adafruit_LSM303DLH_Mag_Unified();
  if (!_accel || !_mag) {
    teardown();
    return false;
  }

  const uint8_t accel_addr =
      _address == 0 ? kLsm303dlhAccelDefaultAddr : (uint8_t)_address;

  WS_DEBUG_PRINT("[drvLsm303dlh] Initialising accel @ 0x");
  WS_DEBUG_PRINTHEX(accel_addr);
  WS_DEBUG_PRINTLN("...");
  if (!_accel->begin(accel_addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm303dlh] Failed to initialise accelerometer");
    teardown();
    return false;
  }
  _accel->setRange(LSM303_RANGE_2G);
  _accel->setMode(LSM303_MODE_HIGH_RESOLUTION);

  WS_DEBUG_PRINT("[drvLsm303dlh] Initialising magnetometer @ 0x");
  WS_DEBUG_PRINTHEX(kLsm303dlhMagDefaultAddr);
  WS_DEBUG_PRINTLN("...");
  if (!_mag->begin(kLsm303dlhMagDefaultAddr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm303dlh] Failed to initialise LSM303DLH mag");
    teardown();
    return false;
  }
  _mag->enableAutoRange(true);
  _mag->setMagGain(LSM303_MAGGAIN_1_9);
  _mag->setMagRate(LSM303_MAGRATE_15);

  WS_DEBUG_PRINTLN("[drvLsm303dlh] Sensor initialised successfully");
  return true;
}

bool drvLsm303dlh::computeAccelMagnitude(float &magnitude) {
  if (!_accel) {
    return false;
  }
  sensors_event_t accel_event;
  if (!_accel->getEvent(&accel_event)) {
    return false;
  }
  magnitude = sqrtf(accel_event.acceleration.x * accel_event.acceleration.x +
                    accel_event.acceleration.y * accel_event.acceleration.y +
                    accel_event.acceleration.z * accel_event.acceleration.z);
  return true;
}

bool drvLsm303dlh::getEventRaw(sensors_event_t *rawEvent) {
  float magnitude = 0.0f;
  if (!computeAccelMagnitude(magnitude)) {
    return false;
  }
  rawEvent->data[0] = magnitude;
  return true;
}

bool drvLsm303dlh::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!_accel) {
    return false;
  }
  return _accel->getEvent(accelEvent);
}

bool drvLsm303dlh::getEventMagneticField(sensors_event_t *magEvent) {
  if (!_mag) {
    return false;
  }
  return _mag->getEvent(magEvent);
}

void drvLsm303dlh::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
}
