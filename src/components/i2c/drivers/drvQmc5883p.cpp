/*!
 * @file drvQmc5883p.cpp
 *
 * Driver wrapper for the Adafruit QMC5883P 3-axis magnetometer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
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

  // CUSTOM SETTINGS CANDIDATES (currently hard-coded defaults below; not yet
  // exposed via the v2 properties API):
  //  - Output data rate: setODR(QMC5883P_ODR_10/50/100/200HZ).
  //  - Oversample / downsample ratios: setOSR(...) / setDSR(...) trade noise
  //    for power.
  //  - Full-scale range: setRange(QMC5883P_RANGE_2G/8G/12G/30G).
  //  - Set/reset mode and measurement mode (continuous vs single).
  _qmc->setMode(QMC5883P_MODE_CONTINUOUS);
  _qmc->setODR(QMC5883P_ODR_50HZ);
  _qmc->setOSR(QMC5883P_OSR_4);
  _qmc->setDSR(QMC5883P_DSR_2);
  _qmc->setRange(QMC5883P_RANGE_30G);
  _qmc->setSetResetMode(QMC5883P_SETRESET_ON);

  return true;
}

/*******************************************************************************/
/*!
    @brief    Gets the QMC5883P's magnetic-field magnitude (gauss) as a raw
              event.
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

  float gx, gy, gz;

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

  WS_DEBUG_PRINT("QMC5883P Gauss X: ");
  WS_DEBUG_PRINTVAR(gx);
  WS_DEBUG_PRINT(" Y: ");
  WS_DEBUG_PRINTVAR(gy);
  WS_DEBUG_PRINT(" Z: ");
  WS_DEBUG_PRINTLNVAR(gz);

  // Report the magnitude of the magnetic field vector, in gauss.
  float magnitude_G = sqrtf(gx * gx + gy * gy + gz * gz);
  rawEvent->data[0] = magnitude_G;
  return true;
}
