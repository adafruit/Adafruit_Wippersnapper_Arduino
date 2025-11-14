/*!
 * @file drvLis3mdl.cpp
 *
 * Driver wrapper for the Adafruit LIS3MDL 3-axis magnetometer.
 */

#include "drvLis3mdl.h"

#include <math.h>

/******************************************************************************/
/*! 
    @brief    Destructor for a LIS3MDL sensor.
*/
/******************************************************************************/
drvLis3mdl::~drvLis3mdl() {
  if (_mag) {
    delete _mag;
    _mag = nullptr;
  }
}

/******************************************************************************/
/*! 
    @brief    Initializes the LIS3MDL sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/******************************************************************************/
bool drvLis3mdl::begin() {
  WS_DEBUG_PRINTLN("[drvLis3mdl] Initialising sensor");
  _mag = new Adafruit_LIS3MDL();
  if (!_mag)
    return false;

  if (!_mag->begin_I2C(_address, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLis3mdl] Failed to initialise sensor");
    delete _mag;
    _mag = nullptr;
    return false;
  }

  _mag->setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  _mag->setOperationMode(LIS3MDL_CONTINUOUSMODE);
  _mag->setDataRate(LIS3MDL_DATARATE_80_HZ);
  _mag->setRange(LIS3MDL_RANGE_4_GAUSS);
  _mag->setIntThreshold(500);
  _mag->configInterrupt(true, true, true, true, false, true);

  WS_DEBUG_PRINTLN("[drvLis3mdl] Sensor initialised successfully");
  return true;
}

/******************************************************************************/
/*! 
    @brief    Reads the LIS3MDL's magnetometer event.
    @param    event
              Pointer to the magnetometer event to populate.
    @returns  True if the event was obtained successfully, False otherwise.
*/
/******************************************************************************/
bool drvLis3mdl::readMagEvent(sensors_event_t *event) {
  if (!_mag)
    return false;
  return _mag->getEvent(event);
}

/******************************************************************************/
/*! 
    @brief    Computes the vector magnitude of a magnetometer reading.
    @param    event
              Magnetometer event to evaluate.
    @param    magnitude
              Reference to store the computed magnitude (micro Tesla).
    @returns  True if the magnitude was computed successfully.
*/
/******************************************************************************/
bool drvLis3mdl::computeMagnitude(const sensors_event_t &event,
                                  float &magnitude) {
  magnitude =
      sqrtf(event.magnetic.x * event.magnetic.x +
            event.magnetic.y * event.magnetic.y +
            event.magnetic.z * event.magnetic.z);
  return true;
}

/******************************************************************************/
/*! 
    @brief    Gets the LIS3MDL's raw sensor event (magnitude stored in data[0]).
    @param    rawEvent
              Pointer to the sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLis3mdl::getEventRaw(sensors_event_t *rawEvent) {
  sensors_event_t magEvent;
  if (!readMagEvent(&magEvent)) {
    return false;
  }

  float magnitude = 0;
  computeMagnitude(magEvent, magnitude);
  rawEvent->data[0] = magnitude;
  return true;
}

/******************************************************************************/
/*! 
    @brief    Gets the LIS3MDL's boolean sensor event.
    @param    booleanEvent
              Pointer to the sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLis3mdl::getEventBoolean(sensors_event_t *booleanEvent) {
  // TODO(#offline-imu-compass): Reuse the raw magnitude once compass headings
  // are supported. For now, magnetometers do not emit boolean events.
  booleanEvent->data[0] = 0.0f;
  return true;
}

/******************************************************************************/
/*! 
    @brief    Gets the LIS3MDL's magnetometer sensor event (x,y,z in uTesla).
    @param    magEvent
              Pointer to the magnetometer sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/******************************************************************************/
bool drvLis3mdl::getEventMagneticField(sensors_event_t *magEvent) {
  return readMagEvent(magEvent);
}

void drvLis3mdl::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
}
