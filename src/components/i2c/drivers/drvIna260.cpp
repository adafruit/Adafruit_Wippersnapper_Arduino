/*!
 * @file drvIna260.cpp
 *
 * Device driver for the INA260 DC Current and Voltage Monitor
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

#include "drvIna260.h"
#include <Adafruit_INA260.h>

/*******************************************************************************/
/*!
    @brief    Destructor for an INA260 sensor.
*/
/*******************************************************************************/
drvIna260::~drvIna260() {
  if (_ina260) {
    delete _ina260;
    _ina260 = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA260 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna260::begin() {
  _ina260 = new Adafruit_INA260();
  if (!_ina260->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("INA260 failed to initialise!");
    return false;
  }

  _ina260->setAveragingCount(INA260_COUNT_16);
  _ina260->setVoltageConversionTime(INA260_TIME_140_us);
  _ina260->setCurrentConversionTime(INA260_TIME_140_us);

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
bool drvIna260::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina260->readBusVoltage();
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
bool drvIna260::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina260->readCurrent();
  return true;
}

void drvIna260::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT;
}