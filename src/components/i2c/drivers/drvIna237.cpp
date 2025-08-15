/*!
 * @file drvIna237.cpp
 *
 * Device driver for the INA237 High Precision DC Current and Voltage Monitor
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

#include "drvIna237.h"
#include <Adafruit_INA237.h>

/*******************************************************************************/
/*!
    @brief    Destructor for an INA237 sensor.
*/
/*******************************************************************************/
drvIna237::~drvIna237() {
  if (_ina237) {
    delete _ina237;
    _ina237 = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA237 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna237::begin() {
  _ina237 = new Adafruit_INA237();
  if (!_ina237->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("INA237 failed to initialise!");
    return false;
  }

  _ina237->setShunt(0.015, 10.0);
  if (_ina237->getCurrentConversionTime() != INA2XX_TIME_280_us) {
    _ina237->setCurrentConversionTime(INA2XX_TIME_280_us);
  }
  if (_ina237->getAveragingCount() != INA2XX_COUNT_16) {
    _ina237->setAveragingCount(INA2XX_COUNT_16);
  }
  if (_ina237->getVoltageConversionTime() != INA2XX_TIME_150_us) {
    _ina237->setVoltageConversionTime(INA2XX_TIME_150_us);
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
bool drvIna237::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina237->getBusVoltage_V();
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
bool drvIna237::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina237->getCurrent_mA();
  return true;
}

void drvIna237::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 2;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  _default_sensor_types[1] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT;
}