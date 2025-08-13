/*!
 * @file drvIna228.cpp
 *
 * Device driver for the INA228 High Precision DC Current and Voltage Monitor
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

#include "drvIna228.h"
#include <Adafruit_INA228.h>

/*******************************************************************************/
/*!
    @brief    Destructor for an INA228 sensor.
*/
/*******************************************************************************/
drvIna228::~drvIna228() {
  if (_ina228) {
    delete _ina228;
    _ina228 = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA228 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::begin() {
  _ina228 = new Adafruit_INA228();
  if (!_ina228->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("INA228 failed to initialise!");
    return false;
  }
  
  _ina228->setShunt(0.015, 10.0);
  if (_ina228->getCurrentConversionTime() != INA228_TIME_280_us) {
    _ina228->setCurrentConversionTime(INA228_TIME_280_us);
  }
  if (_ina228->getAveragingCount() != INA228_COUNT_16) {
    _ina228->setAveragingCount(INA228_COUNT_16);
  }
  if (_ina228->getVoltageConversionTime() != INA228_TIME_150_us) {
    _ina228->setVoltageConversionTime(INA228_TIME_150_us);
  }
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
bool drvIna228::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina228->getBusVoltage_V();
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
bool drvIna228::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina228->getCurrent_mA();
  return true;
}

void drvIna228::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT;
}