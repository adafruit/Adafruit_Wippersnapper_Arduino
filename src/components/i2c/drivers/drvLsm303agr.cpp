/*!
 * @file drvLsm303agr.cpp
 *
 * Driver wrapper for the Adafruit LSM303AGR combo sensor.
 */

#include "drvLsm303agr.h"

#include <math.h>

namespace {
constexpr uint8_t kLsm303agrAccelDefaultAddr = LSM303_ADDRESS_ACCEL;
constexpr uint8_t kLsm303agrMagDefaultAddr = 0x1E; // LIS2MDL address
}

/******************************************************************************/
/*! 
    @brief  Destructor for the LSM303AGR driver wrapper.
*/
/******************************************************************************/
drvLsm303agr::~drvLsm303agr() { teardown(); }

/******************************************************************************/
/*! 
    @brief  Releases any allocated accelerometer or magnetometer instances.
*/
/******************************************************************************/
void drvLsm303agr::teardown() {
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
/*! 
    @brief  Initializes the LSM303AGR accelerometer and LIS2MDL magnetometer.
    @returns True if initialization succeeded, False otherwise.
*/
/******************************************************************************/
bool drvLsm303agr::begin() {
  teardown();

  _accel = new Adafruit_LSM303_Accel_Unified();
  _mag = new Adafruit_LIS2MDL();
  if (!_accel || !_mag) {
    teardown();
    return false;
  }

  const uint8_t accel_addr =
      _address == 0 ? kLsm303agrAccelDefaultAddr : (uint8_t)_address;

  WS_DEBUG_PRINT("[drvLsm303agr] Initialising accel @ 0x");
  WS_DEBUG_PRINTHEX(accel_addr);
  WS_DEBUG_PRINTLN("...");
  if (!_accel->begin(accel_addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm303agr] Failed to initialise accelerometer");
    teardown();
    return false;
  }
  _accel->setRange(LSM303_RANGE_2G);
  _accel->setMode(LSM303_MODE_HIGH_RESOLUTION);

  WS_DEBUG_PRINT("[drvLsm303agr] Initialising magnetometer @ 0x");
  WS_DEBUG_PRINTHEX(kLsm303agrMagDefaultAddr);
  WS_DEBUG_PRINTLN("...");
  if (!_mag->begin(kLsm303agrMagDefaultAddr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm303agr] Failed to initialise LIS2MDL");
    teardown();
    return false;
  }
  _mag->setDataRate(LIS2MDL_RATE_50_HZ);
  _mag->enableInterrupts(false);
  _mag->interruptsActiveHigh(false);

  WS_DEBUG_PRINTLN("[drvLsm303agr] Sensor initialised successfully");
  return true;
}

/******************************************************************************/
/*! 
  @brief  Computes the magnitude of the accelerometer vector.
  @param  magnitude Reference to store the computed m/s^2 value.
  @returns True if the accelerometer event was retrieved successfully.
*/
/******************************************************************************/
bool drvLsm303agr::computeAccelMagnitude(float &magnitude) {
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

/******************************************************************************/
/*! 
  @brief  Fills the raw event with the accelerometer magnitude in data[0].
  @param  rawEvent Pointer to the destination sensor event.
  @returns True if the magnitude was computed successfully.
*/
/******************************************************************************/
bool drvLsm303agr::getEventRaw(sensors_event_t *rawEvent) {
  float magnitude = 0.0f;
  if (!computeAccelMagnitude(magnitude)) {
    return false;
  }
  rawEvent->data[0] = magnitude;
  return true;
}

/******************************************************************************/
/*! 
  @brief  Retrieves the 3-axis accelerometer event.
  @param  accelEvent Pointer to the destination sensor event.
  @returns True if the event was populated successfully.
*/
/******************************************************************************/
bool drvLsm303agr::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!_accel) {
    return false;
  }
  return _accel->getEvent(accelEvent);
}

/******************************************************************************/
/*! 
  @brief  Retrieves the 3-axis magnetic field event.
  @param  magEvent Pointer to the destination sensor event.
  @returns True if the event was populated successfully.
*/
/******************************************************************************/
bool drvLsm303agr::getEventMagneticField(sensors_event_t *magEvent) {
  if (!_mag) {
    return false;
  }
  return _mag->getEvent(magEvent);
}

/******************************************************************************/
/*! 
  @brief  Registers the driver's default accelerometer and magnetometer types.
*/
/******************************************************************************/
void drvLsm303agr::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
}
