/*!
 * @file drvLsm9ds1.cpp
 *
 * Driver wrapper for the Adafruit LSM9DS1 9-DOF IMU.
 *
 */

#include "drvLsm9ds1.h"

#include <Adafruit_LSM9DS1.h>
#include <math.h>

/******************************************************************************/
/*!
    @brief    Destructor for a LSM9DS1 sensor.
*/
/******************************************************************************/
drvLsm9ds1::~drvLsm9ds1() {
  if (_lsm) {
    delete _lsm;
    _lsm = nullptr;
  }
}

/******************************************************************************/
/*!
    @brief    Initializes the LSM9DS1 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/******************************************************************************/
bool drvLsm9ds1::begin() {
  _lsm = new Adafruit_LSM9DS1(_i2c);
  // Consumes I2C Addresses 0x1E and 0x6B
  if (!_lsm->begin()) {
    WS_DEBUG_PRINTLN("LSM9DS1 failed to initialise!");
    return false;
  }

  // Mirror the configuration used by the reference example
  _lsm->setupAccel(_lsm->LSM9DS1_ACCELRANGE_4G,
                   _lsm->LSM9DS1_ACCELDATARATE_119HZ);
  _lsm->setupMag(_lsm->LSM9DS1_MAGGAIN_4GAUSS);
  _lsm->setupGyro(_lsm->LSM9DS1_GYROSCALE_245DPS);

  return true;
}

bool drvLsm9ds1::readAllEvents(sensors_event_t *accel, sensors_event_t *mag,
                               sensors_event_t *gyro, sensors_event_t *temp) {
  if (!_lsm) {
    return false;
  }
  
  return _lsm->getEvent(accel, mag, gyro, temp);
}

/******************************************************************************/
/*!
    @brief    Gets the LSM9DS1's raw sensor event.
    @param    rawEvent
              Pointer to the sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLsm9ds1::getEventRaw(sensors_event_t *rawEvent) {
  // TODO: Not yet, but eventually migrate to providing the temperatre data here
  WS_DEBUG_PRINTLN("[drvLsm9ds1] Getting raw event...");
  sensors_event_t accel, mag, gyro, temp;
  if (!readAllEvents(&accel, &mag, &gyro, &temp)) {
    return false;
  }

  float mag_accel = sqrtf(accel.acceleration.x * accel.acceleration.x +
                          accel.acceleration.y * accel.acceleration.y +
                          accel.acceleration.z * accel.acceleration.z);
  rawEvent->data[0] = mag_accel;
  WS_DEBUG_PRINT("[drvLsm9ds1] Raw magnitude: ");
  WS_DEBUG_PRINTLN(mag_accel);
  return true;
}

/******************************************************************************/
/*!
    @brief    Gets the LSM9DS1's accelerometer sensor event (x,y,z in m/s^2).
    @param    accelEvent
              Pointer to the accelerometer sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLsm9ds1::getEventAccelerometer(sensors_event_t *accelEvent) {
  WS_DEBUG_PRINTLN("[drvLsm9ds1] Getting accelerometer event...");
  sensors_event_t mag, gyro, temp;
  if (!readAllEvents(accelEvent, &mag, &gyro, &temp)) {
    return false;
  }
  return true;
}

/******************************************************************************/
/*!
    @brief    Gets the LSM9DS1's temperature sensor event (not necessarily *C).
    @param    tempEvent
              Pointer to the temperature sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLsm9ds1::getEventAmbientTemp(sensors_event_t *tempEvent) {
  WS_DEBUG_PRINTLN("[drvLsm9ds1] Getting temperature event...");
  sensors_event_t accel, mag, gyro;
  if (!readAllEvents(&accel, &mag, &gyro, tempEvent)) {
    return false;
  }
  return true;
}

/******************************************************************************/
/*!
    @brief    Gets the LSM9DS1's gyroscope sensor event (x,y,z in rad/s).
    @param    gyroEvent
              Pointer to the gyroscope sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLsm9ds1::getEventGyroscope(sensors_event_t *gyroEvent) {
  WS_DEBUG_PRINTLN("[drvLsm9ds1] Getting gyroscope event...");
  sensors_event_t accel, mag, temp;
  if (!readAllEvents(&accel, &mag, gyroEvent, &temp)) {
    return false;
  }
  return true;
}

/******************************************************************************/
/*!
    @brief    Gets the LSM9DS1's magnetometer sensor event (x,y,z in uT).
    @param    magEvent
              Pointer to the magnetometer sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLsm9ds1::getEventMagneticField(sensors_event_t *magEvent) {
  WS_DEBUG_PRINTLN("[drvLsm9ds1] Getting magnetometer event...");
  sensors_event_t accel, gyro, temp;
  if (!readAllEvents(&accel, magEvent, &gyro, &temp)) {
    return false;
  }
  return true;
}

void drvLsm9ds1::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 4;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
  _default_sensor_types[2] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_GYROSCOPE;
  _default_sensor_types[3] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
}
