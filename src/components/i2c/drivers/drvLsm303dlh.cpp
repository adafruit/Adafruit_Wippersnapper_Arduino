/*!
 * @file drvLsm303dlh.cpp
 *
 * Driver wrapper for the legacy Adafruit LSM303DLH combo sensor.
 */

#include "drvLsm303dlh.h"

#include <math.h>

#define LSM303DLH_ACCEL_DEFAULT_ADDR LSM303_ADDRESS_ACCEL
#define LSM303DLH_MAG_DEFAULT_ADDR 0x1E

/******************************************************************************/
/*!
  @brief  Destructor for the legacy LSM303DLH driver wrapper.
*/
/******************************************************************************/
drvLsm303dlh::~drvLsm303dlh() { teardown(); }

/******************************************************************************/
/*!
  @brief  Releases any allocated accelerometer or magnetometer instances.
*/
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
/*!
  @brief  Initializes the LSM303DLH accelerometer and magnetometer helpers.
  @returns True if initialization succeeded, False otherwise.
*/
/******************************************************************************/
bool drvLsm303dlh::begin() {
  WS_DEBUG_PRINTLN("[drvLsm303dlh] Initializing LSM303DLH driver...");
  WS_PRINTER.flush();

  WS_DEBUG_PRINTLN("[drvLsm303dlh] Tearing down any existing sensor instances...");
  WS_PRINTER.flush();
  teardown();

  WS_DEBUG_PRINTLN("[drvLsm303dlh] Creating new sensor instances...");
  WS_PRINTER.flush();
  _accel = new Adafruit_LSM303_Accel_Unified();
  WS_DEBUG_PRINTLN("[drvLsm303dlh] Created accelerometer instance, DLH mag next");
  WS_PRINTER.flush();
  _mag = new Adafruit_LSM303DLH_Mag_Unified();
  WS_DEBUG_PRINTLN("[drvLsm303dlh] Created magnetometer instance");
  WS_PRINTER.flush();
  if (!_accel || !_mag) {
    teardown();
    return false;
  }
  WS_DEBUG_PRINT("[drvLsm303dlh] Initialising accelerometer @ 0x");
  WS_DEBUG_PRINTHEX(LSM303DLH_ACCEL_DEFAULT_ADDR);
  WS_DEBUG_PRINTLN("...");
  if (!_accel->begin(LSM303DLH_ACCEL_DEFAULT_ADDR, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLsm303dlh] Failed to initialise accelerometer");
    teardown();
    return false;
  }
  _accel->setRange(LSM303_RANGE_2G);
  _accel->setMode(LSM303_MODE_HIGH_RESOLUTION);

  WS_DEBUG_PRINT("[drvLsm303dlh] Initialising magnetometer @ 0x");
  WS_DEBUG_PRINTHEX(LSM303DLH_MAG_DEFAULT_ADDR);
  WS_DEBUG_PRINTLN("...");
  if (!_mag->begin(LSM303DLH_MAG_DEFAULT_ADDR, _i2c)) {
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

/******************************************************************************/
/*!
  @brief  Computes the magnitude of the accelerometer vector.
  @param  magnitude Reference to store the computed m/s^2 value.
  @returns True if the accelerometer event was retrieved successfully.
*/
/******************************************************************************/
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

/******************************************************************************/
/*!
  @brief  Fills the raw event with the accelerometer magnitude in data[0].
  @param  rawEvent Pointer to the destination sensor event.
  @returns True if the magnitude was computed successfully.
*/
/******************************************************************************/
bool drvLsm303dlh::getEventRaw(sensors_event_t *rawEvent) {
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
bool drvLsm303dlh::getEventAccelerometer(sensors_event_t *accelEvent) {
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
bool drvLsm303dlh::getEventMagneticField(sensors_event_t *magEvent) {
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
void drvLsm303dlh::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
}
