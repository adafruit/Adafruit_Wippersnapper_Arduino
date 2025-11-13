/*!
 * @file drvLis3dh.cpp
 *
 * Driver wrapper for the Adafruit LIS3DH 3-axis accelerometer.
 *
 */

#include "drvLis3dh.h"

#include <Adafruit_LIS3DH.h>
#include <math.h>

/******************************************************************************/
/*!
    @brief    Destructor for a LIS3DH sensor.
*/
/******************************************************************************/
drvLis3dh::~drvLis3dh() {
  if (_lis) {
    delete _lis;
    _lis = nullptr;
  }
}

/******************************************************************************/
/*!
    @brief    Initializes the LIS3DH sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/******************************************************************************/
bool drvLis3dh::begin() {
  // allocate LIS3DH instance with the I2C bus
  _lis = new Adafruit_LIS3DH(_i2c);

  // Attempt to initialize with provided address
  if (!_lis->begin(_address)) {
    WS_DEBUG_PRINTLN("LIS3DH failed to initialise!");
    return false;
  }

  // Set a reasonable range and data rate (use library enums)
  _lis->setRange(LIS3DH_RANGE_2_G); // 2G range
  // Note: some Adafruit_LIS3DH variants offer setDataRate; if present this
  // keeps defaults. Keep configuration minimal to avoid API mismatches.

  return true;
}

/******************************************************************************/
/*!
    @brief    Gets the LIS3DH's raw sensor event.
    @param    rawEvent
              Pointer to the sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLis3dh::getEventRaw(sensors_event_t *rawEvent) {
  if (!_lis)
    return false;

  // Read an Adafruit_Sensor compatible event from the device
  sensors_event_t event;
  _lis->getEvent(&event);

  // Calculate magnitude of the acceleration vector (m/s^2) and store in
  // event->data[0] to be consistent with other drivers that expose "raw"
  float mag = sqrtf(event.acceleration.x * event.acceleration.x +
                     event.acceleration.y * event.acceleration.y +
                     event.acceleration.z * event.acceleration.z);
  rawEvent->data[0] = mag;
  return true;
}

/******************************************************************************/
/*!
    @brief    Gets the LIS3DH's accelerometer sensor event (x,y,z in m/s^2).
    @param    accelEvent
              Pointer to the accelerometer sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLis3dh::getEventAccelerometer(sensors_event_t *accelEvent) {
  if (!_lis)
    return false;

  // Fill the provided event with sensor data
  _lis->getEvent(accelEvent);
  return true;
}

void drvLis3dh::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
}