/*!
 * @file WipperSnapper_I2C_Driver_QMC5883P.cpp
 *
 * Implementation for the Adafruit QMC5883P magnetometer wrapper.
 */

#include "WipperSnapper_I2C_Driver_QMC5883P.h"

#include <Adafruit_QMC5883P.h>
#include <math.h>

WipperSnapper_I2C_Driver_QMC5883P::WipperSnapper_I2C_Driver_QMC5883P(
    TwoWire *i2c, uint16_t sensorAddress)
    : WipperSnapper_I2C_Driver(i2c, sensorAddress) {}

WipperSnapper_I2C_Driver_QMC5883P::~WipperSnapper_I2C_Driver_QMC5883P() {
  if (_qmc) {
    delete _qmc;
  }
}

bool WipperSnapper_I2C_Driver_QMC5883P::begin() {
  _qmc = new Adafruit_QMC5883P();
  if (!_qmc->begin(_sensorAddress, _i2c)) {
    return false;
  }

  // Set to continuous mode
  _qmc->setMode(QMC5883P_MODE_CONTINUOUS);
  // Set ODR (Output Data Rate) to 50Hz
  _qmc->setODR(QMC5883P_ODR_50HZ);
  // Set OSR (Over Sample Ratio) to 4
  _qmc->setOSR(QMC5883P_OSR_4);
  // Set DSR (Downsample Ratio) to 2
  _qmc->setDSR(QMC5883P_DSR_2);
  // Set Range to 30G
  _qmc->setRange(QMC5883P_RANGE_30G);
  // Set SetReset mode to On
  _qmc->setSetResetMode(QMC5883P_SETRESET_ON);

  return true;
}

bool WipperSnapper_I2C_Driver_QMC5883P::getEventRaw(sensors_event_t *magEvent) {
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
    WS_DEBUG_PRINT("Raw X: ");
    WS_DEBUG_PRINTLN(x);
    WS_DEBUG_PRINT("Raw Y: ");
    WS_DEBUG_PRINTLN(y);
    WS_DEBUG_PRINT("Raw Z: ");
    WS_DEBUG_PRINTLN(z);
    return false;
  } else {
    WS_DEBUG_PRINT("Gauss X: ");
    WS_DEBUG_PRINTLN(gx);
    WS_DEBUG_PRINT("Gauss Y: ");
    WS_DEBUG_PRINTLN(gy);
    WS_DEBUG_PRINT("Gauss Z: ");
    WS_DEBUG_PRINTLN(gz);
  }

  // Check for overflow
  if (_qmc->isOverflow()) {
    WS_DEBUG_PRINTLN("QMC5883P data overflow - skipping reading");
    return false;
  }

  // Calculate magnitude in Gauss
  float magnitude_G = sqrtf(gx * gx + gy * gy + gz * gz);
  magEvent->data[0] = magnitude_G;
  return true;
}
