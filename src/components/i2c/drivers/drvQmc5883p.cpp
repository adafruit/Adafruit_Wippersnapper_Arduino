/*!
 * @file drvQmc5883p.cpp
 *
 * Driver wrapper for the Adafruit QMC5883P 3-axis magnetometer.
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

#include "drvQmc5883p.h"
#include <Adafruit_QMC5883P.h>

/*******************************************************************************/
/*!
    @brief    Destructor for a QMC5883P sensor.
*/
/*******************************************************************************/
drvQmc5883p::~drvQmc5883p() {
  if (_qmc) {
    delete _qmc;
    _qmc = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the QMC5883P sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::begin() {
  _qmc = new Adafruit_QMC5883P();
  if (!_qmc->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("QMC5883P failed to initialise!");
    return false;
  }

  // Configure sensor with same settings as example sketch
  // Set to continuous mode for continuous reading
  _qmc->setMode(QMC5883P_MODE_CONTINUOUS);

  // Set ODR (Output Data Rate) to 50Hz
  _qmc->setODR(QMC5883P_ODR_50HZ);

  // Set OSR (Over Sample Ratio) to 4
  _qmc->setOSR(QMC5883P_OSR_4);

  // Set DSR (Downsample Ratio) to 2
  _qmc->setDSR(QMC5883P_DSR_2);

  // Set Range to 30G for maximum sensitivity range
  _qmc->setRange(QMC5883P_RANGE_30G);

  // Set SetReset mode to On
  _qmc->setSetResetMode(QMC5883P_SETRESET_ON);

  return true;
}

/*******************************************************************************/
/*!
    @brief    Gets the QMC5883P's magnetometer sensor event as raw magnitude.
    @param    rawEvent
              Pointer to the magnetometer sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::getEventRaw(sensors_event_t *rawEvent) {
  // Check if data is ready before reading
  if (!_qmc->isDataReady()) {
    return false;
  }

  int16_t x, y, z;
  float gx, gy, gz;

  // Get raw magnetic data
  if (!_qmc->getRawMagnetic(&x, &y, &z)) {
    WS_DEBUG_PRINTLN("Failed to read raw magnetic data");
    return false;
  }

  // Get Gauss field data
  if (!_qmc->getGaussField(&gx, &gy, &gz)) {
    WS_DEBUG_PRINTLN("Failed to read Gauss field data");
    return false;
  }

  // Check for overflow
  if (_qmc->isOverflow()) {
    WS_DEBUG_PRINTLN("QMC5883P data overflow - skipping reading");
    return false;
  }

  // Calculate magnitude in Gauss
  float magnitude_G = sqrtf(gx * gx + gy * gy + gz * gz);
  rawEvent->data[0] = magnitude_G;
  return true;
}

/*******************************************************************************/
/*!
    @brief    Gets the QMC5883P's magnetic field vector.
    @param    magneticEvent
              Pointer to the magnetic field sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::getEventMagneticField(sensors_event_t *magneticEvent) {
  // Check if data is ready before reading
  if (!_qmc->isDataReady()) {
    return false;
  }

  int16_t x, y, z;
  float gx, gy, gz;

  // Get raw magnetic data
  if (!_qmc->getRawMagnetic(&x, &y, &z)) {
    WS_DEBUG_PRINTLN("Failed to read raw magnetic data");
    return false;
  }

  // Get Gauss field data
  if (!_qmc->getGaussField(&gx, &gy, &gz)) {
    WS_DEBUG_PRINTLN("Failed to read Gauss field data");
    return false;
  }

  // Check for overflow
  if (_qmc->isOverflow()) {
    WS_DEBUG_PRINTLN("QMC5883P data overflow - skipping reading");
    return false;
  }

  // Convert from Gauss to microTesla (1 Gauss = 100 microTesla)
  magneticEvent->magnetic.x = gx * 100.0f;
  magneticEvent->magnetic.y = gy * 100.0f;
  magneticEvent->magnetic.z = gz * 100.0f;

  return true;
}

void drvQmc5883p::ConfigureDefaultSensorTypes() {
  _default_sensor_types_count = 1;
  _default_sensor_types[0] =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
}