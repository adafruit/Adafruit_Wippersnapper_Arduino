/*!
 * @file WipperSnapper_I2C_Driver_INA237.cpp
 *
 * Device driver implementation for the INA237 DC Current and Voltage Monitor
 * (Avoids import conflict with INA260 typedef enum _mode etc)
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

#include "WipperSnapper_I2C_Driver_INA237.h"
#include "Wippersnapper.h"
#include <Adafruit_INA237.h>

/*******************************************************************************/
/*!
    @brief    Constructor for a INA237 sensor.
    @param    i2c
              The I2C interface.
    @param    sensorAddress
              The 7-bit I2C address of the sensor.
*/
/*******************************************************************************/
WipperSnapper_I2C_Driver_INA237::WipperSnapper_I2C_Driver_INA237(
    TwoWire *i2c, uint16_t sensorAddress)
    : WipperSnapper_I2C_Driver(i2c, sensorAddress), _ina237(nullptr) {
  _i2c = i2c;
  _sensorAddress = sensorAddress;
}

/*******************************************************************************/
/*!
    @brief    Destructor for an INA237 sensor.
*/
/*******************************************************************************/
WipperSnapper_I2C_Driver_INA237::~WipperSnapper_I2C_Driver_INA237() {
  delete _ina237;
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA237 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_I2C_Driver_INA237::begin() {
  _ina237 = new Adafruit_INA237();
  if (!_ina237->begin(_sensorAddress, _i2c)) {
    return false;
  }

  // Configuration based on INA237 example sketch
  // Set default shunt resistance and maximum current
  // Default 0.015 ohm shunt, 10A max current
  _ina237->setShunt(0.015, 10.0);

  // Set averaging for better accuracy (16 samples)
  _ina237->setAveragingCount(INA2XX_COUNT_16);

  // Set conversion times as per example
  _ina237->setVoltageConversionTime(INA2XX_TIME_150_us);
  _ina237->setCurrentConversionTime(INA2XX_TIME_280_us);

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
bool WipperSnapper_I2C_Driver_INA237::getEventVoltage(
    sensors_event_t *voltageEvent) {
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
bool WipperSnapper_I2C_Driver_INA237::getEventCurrent(
    sensors_event_t *currentEvent) {
  currentEvent->current = _ina237->getCurrent_mA();
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
bool WipperSnapper_I2C_Driver_INA237::getEventRaw(sensors_event_t *powerEvent) {
  powerEvent->data[0] = _ina237->getPower_mW();
  return true;
}