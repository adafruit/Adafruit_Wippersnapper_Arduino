/*!
 * @file drvLis2mdl.cpp
 *
 * Driver wrapper for the Adafruit LIS2MDL 3-axis magnetometer.
 */

#include "drvLis2mdl.h"

#include <math.h>

#define LIS2MDL_DEFAULT_ADDR 0x1E

/******************************************************************************/
/*!
    @brief  Destructor for the LIS2MDL driver wrapper.
*/
/******************************************************************************/
drvLis2mdl::~drvLis2mdl() {
  if (_mag) {
    delete _mag;
    _mag = nullptr;
  }
}

/******************************************************************************/
/*!
    @brief  Initializes the LIS2MDL sensor and begins I2C.
    @returns True if initialized successfully, False otherwise.
*/
/******************************************************************************/
bool drvLis2mdl::begin() {
  WS_DEBUG_PRINTLN("[drvLis2mdl] Initialising sensor");

  if (_mag) {
    delete _mag;
    _mag = nullptr;
  }

  _mag = new Adafruit_LIS2MDL();
  if (!_mag) {
    return false;
  }

  const uint8_t addr =
      _address == 0 ? LIS2MDL_DEFAULT_ADDR : static_cast<uint8_t>(_address);
  if (!_mag->begin(addr, _i2c)) {
    WS_DEBUG_PRINTLN("[drvLis2mdl] Failed to initialise sensor");
    delete _mag;
    _mag = nullptr;
    return false;
  }

  _mag->setDataRate(LIS2MDL_RATE_50_HZ);
  _mag->enableInterrupts(false);
  _mag->interruptsActiveHigh(false);

  WS_DEBUG_PRINTLN("[drvLis2mdl] Sensor initialised successfully");
  return true;
}

/******************************************************************************/
/*!
    @brief  Reads the LIS2MDL's magnetometer event.
    @param  event Pointer to the magnetometer event to populate.
    @returns True if the event was obtained successfully.
*/
/******************************************************************************/
bool drvLis2mdl::readMagEvent(sensors_event_t *event) {
  if (!_mag) {
    return false;
  }
  return _mag->getEvent(event);
}

/******************************************************************************/
/*!
    @brief  Computes the vector magnitude of a magnetometer reading.
    @param  event Magnetometer event to evaluate.
    @param  magnitude Reference to store the computed magnitude (micro Tesla).
    @returns True if the magnitude was computed successfully.
*/
/******************************************************************************/
bool drvLis2mdl::computeMagnitude(const sensors_event_t &event,
                                  float &magnitude) {
  magnitude = sqrtf(event.magnetic.x * event.magnetic.x +
                    event.magnetic.y * event.magnetic.y +
                    event.magnetic.z * event.magnetic.z);
  return true;
}

/******************************************************************************/
/*!
    @brief  Gets the LIS2MDL's raw sensor event (magnitude stored in data[0]).
    @param  rawEvent Pointer to the sensor event.
    @returns True if the sensor event was obtained successfully.
*/
/******************************************************************************/
bool drvLis2mdl::getEventRaw(sensors_event_t *rawEvent) {
  sensors_event_t magEvent;
  if (!readMagEvent(&magEvent)) {
    return false;
  }

  float magnitude = 0.0f;
  computeMagnitude(magEvent, magnitude);
  rawEvent->data[0] = magnitude;
  return true;
}

/******************************************************************************/
/*!
    @brief  Gets the LIS2MDL's boolean sensor event.
    @param  booleanEvent Pointer to the sensor event.
    @returns True once the placeholder value has been populated.
*/
/******************************************************************************/
bool drvLis2mdl::getEventBoolean(sensors_event_t *booleanEvent) {
  // Magnetometers do not emit boolean events; reserve the field for future use.
  booleanEvent->data[0] = 0.0f;
  return true;
}

/******************************************************************************/
/*!
    @brief  Gets the LIS2MDL's magnetometer sensor event (x,y,z in microTesla).
    @param  magEvent Pointer to the magnetometer sensor event.
    @returns True if the sensor event was obtained successfully.
*/
/******************************************************************************/
bool drvLis2mdl::getEventMagneticField(sensors_event_t *magEvent) {
  return readMagEvent(magEvent);
}

/******************************************************************************/
/*!
    @brief  Registers the driver's default magnetometer sensor type.
*/
/******************************************************************************/
void drvLis2mdl::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
}
