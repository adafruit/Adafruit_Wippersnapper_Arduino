/*!
 * @file WipperSnapper_I2C_Driver_INA228.cpp
 *
 * Device driver implementation for the INA228 High Precision DC Current and
 * Voltage Monitor (Avoids import conflict with INA260 typedef enum _mode etc)
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

#include "WipperSnapper_I2C_Driver_INA228.h"
#include "Wippersnapper.h"
#include <Adafruit_INA228.h>

/*******************************************************************************/
/*!
    @brief    Constructor for a INA228 sensor.
    @param    i2c
              The I2C interface.
    @param    sensorAddress
              The 7-bit I2C address of the sensor.
*/
/*******************************************************************************/
WipperSnapper_I2C_Driver_INA228::WipperSnapper_I2C_Driver_INA228(
    TwoWire *i2c, uint16_t sensorAddress)
    : WipperSnapper_I2C_Driver(i2c, sensorAddress), _ina228(nullptr) {
  _i2c = i2c;
  _sensorAddress = sensorAddress;
}

/*******************************************************************************/
/*!
    @brief    Destructor for an INA228 sensor.
*/
/*******************************************************************************/
WipperSnapper_I2C_Driver_INA228::~WipperSnapper_I2C_Driver_INA228() {
  delete _ina228;
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA228 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_I2C_Driver_INA228::begin() {
  _ina228 = new Adafruit_INA228();
  if (!_ina228->begin(_sensorAddress, _i2c)) {
    return false;
  }

  // Default shunt: 0.015 ohm, 10A max current
  _ina228->setShunt(0.015, 10.0);

  _ina228->setAveragingCount(INA228_COUNT_16);
  _ina228->setVoltageConversionTime(INA228_TIME_150_us);
  _ina228->setCurrentConversionTime(INA228_TIME_280_us);

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
bool WipperSnapper_I2C_Driver_INA228::getEventVoltage(
    sensors_event_t *voltageEvent) {
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
bool WipperSnapper_I2C_Driver_INA228::getEventCurrent(
    sensors_event_t *currentEvent) {
  currentEvent->current = _ina228->getCurrent_mA();
  return true;
}

/**
 * @brief   Get the raw (power) sensor event.
 *
 * @param   powerEvent  Pointer to the power sensor event.
 *
 * @returns True if the sensor event was obtained successfully, False
 * otherwise.
 */
bool WipperSnapper_I2C_Driver_INA228::getEventRaw(sensors_event_t *powerEvent) {
  powerEvent->data[0] = _ina228->getPower_mW();
  return true;
}
