/*!
 * @file drvIna238.cpp
 *
 * Device driver for the INA238 High Precision DC Current and Voltage Monitor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include "drvIna238.h"
#include <Adafruit_INA238.h>

/*******************************************************************************/
/*!
    @brief    Destructor for an INA238 sensor.
*/
/*******************************************************************************/
drvIna238::~drvIna238() {
  if (_ina238) {
    delete _ina238;
    _ina238 = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA238 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna238::begin() {
  _ina238 = new Adafruit_INA238();
  if (!_ina238->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("INA238 failed to initialise!");
    return false;
  }
  // TODO: use setCalibration()

  return true;
}

/*******************************************************************************/
/*!
    @brief    Reads a voltage sensor and converts the
              reading into the expected SI unit.
    @param    voltageEvent
              voltage sensor reading, in volts.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/*******************************************************************************/
bool drvIna238::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina238->getBusVoltage_V();
  return true;
}

/**
 * @brief   Get the current sensor event.
 *
 * @param   currentEvent  Pointer to the current sensor event.
 *
 * @returns True if the sensor event was obtained successfully, False
 * otherwise.
 */
bool drvIna238::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina238->getCurrent_mA();
  return true;
}

void drvIna238::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT;
}