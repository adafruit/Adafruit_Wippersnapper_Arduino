/*!
 * @file drvLsm6dso32.cpp
 *
 * Driver wrapper for the Adafruit LSM6DSO32 6-DoF IMU.
 */

#include "drvLsm6dso32.h"

/******************************************************************************/
drvLsm6dso32::~drvLsm6dso32() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }
}

/******************************************************************************/
/*!
    @brief  Checks whether a part driven as an LSM6DSO32 is really an LSM6DSOX.
            Both report WHO_AM_I 0x6C, but every accel range code maps to half
            the full-scale on the DSOX, so at rest the DSO32 scaling reads the
            gravity vector as ~2 g instead of ~1 g. Uses only data reads after
            the configuration begin() writes anyway, and the vector magnitude,
            so it does not depend on orientation. Needs the part to be still
            (gyro near zero, steady magnitude); otherwise it is inconclusive.
    @returns True only if the part is still and reads ~2 g.
*/
/******************************************************************************/
bool drvLsm6dso32::isDsoxScale() {
  const float G = SENSORS_GRAVITY_STANDARD;
  const uint8_t N = 9;
  float mag[N];
  sensors_event_t a, g, t;
  delay(30); // let the range change settle (library default ODR 104 Hz)
  for (uint8_t i = 0; i < N; i++) {
    if (!_imu->getEvent(&a, &g, &t)) {
      return false;
    }
    float gyro =
        sqrtf(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z);
    if (gyro > 0.1f) { // rad/s: moving, gyro scaling is the same on both
      WS_DEBUG_PRINTLN("[drvLsm6dso32] Moving, DSO32/DSOX check skipped");
      return false;
    }
    mag[i] = sqrtf(a.acceleration.x * a.acceleration.x +
                   a.acceleration.y * a.acceleration.y +
                   a.acceleration.z * a.acceleration.z);
    delay(10); // a fresh sample each time at 104 Hz
  }
  float lo = mag[0], hi = mag[0];
  for (uint8_t i = 1; i < N; i++) {
    lo = min(lo, mag[i]);
    hi = max(hi, mag[i]);
  }
  for (uint8_t i = 1; i < N; i++) { // insertion sort for the median
    float v = mag[i];
    int8_t j = i - 1;
    while (j >= 0 && mag[j] > v) {
      mag[j + 1] = mag[j];
      j--;
    }
    mag[j + 1] = v;
  }
  float median = mag[N / 2];
  WS_DEBUG_PRINT("[drvLsm6dso32] |a| at rest (DSO32 scale): ");
  WS_DEBUG_PRINTLN(median);
  if (hi - lo > 0.1f * G) {
    WS_DEBUG_PRINTLN("[drvLsm6dso32] Unsteady, DSO32/DSOX check skipped");
    return false;
  }
  if (median > 1.6f * G && median < 2.4f * G) {
    return true;
  }
  if (median < 0.6f * G || median > 1.4f * G) {
    WS_DEBUG_PRINTLN("[drvLsm6dso32] Inconclusive, assuming LSM6DSO32");
  }
  return false;
}

/******************************************************************************/
bool drvLsm6dso32::begin() {
  if (_imu) {
    delete _imu;
    _imu = nullptr;
  }

  uint8_t addr = _address == 0 ? LSM6DS_I2CADDR_DEFAULT : (uint8_t)_address;
  WS_DEBUG_PRINT("[drvLsm6dso32] Initialising @ 0x");
  WS_DEBUG_PRINTHEX(addr);
  WS_DEBUG_PRINTLN("...");

  Adafruit_LSM6DSO32 *dso32 = new Adafruit_LSM6DSO32();
  if (!dso32) {
    return false;
  }
  _imu = dso32;
  if (!dso32->begin_I2C(addr, _i2c, getLsmSensorID())) {
    WS_DEBUG_PRINTLN("[drvLsm6dso32] Failed to initialise sensor");
    delete _imu;
    _imu = nullptr;
    return false;
  }
  dso32->setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);

  if (isDsoxScale()) {
    // Same WHO_AM_I, so the LSM6DSOX class accepts it; its range enum
    // gives the correct scaling (8 g here as well).
    WS_DEBUG_PRINTLN("[drvLsm6dso32] Part is an LSM6DSOX, using DSOX scaling");
    delete _imu;
    Adafruit_LSM6DSOX *dsox = new Adafruit_LSM6DSOX();
    _imu = dsox;
    if (!dsox || !dsox->begin_I2C(addr, _i2c, getLsmSensorID())) {
      WS_DEBUG_PRINTLN("[drvLsm6dso32] Failed to initialise LSM6DSOX");
      delete _imu;
      _imu = nullptr;
      return false;
    }
    dsox->setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  }

  _imu->setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  _imu->setAccelDataRate(LSM6DS_RATE_416_HZ);
  _imu->setGyroDataRate(LSM6DS_RATE_416_HZ);
  // _imu->highPassFilter(true, LSM6DS_HPF_ODR_DIV_100);
  _imu->configInt1(false, false, false, false, true);
  // _imu->configInt2(false, false, false);
  _imu->enableWakeup(true);
  // _imu->enablePedometer(true);

  WS_DEBUG_PRINTLN("[drvLsm6dso32] Sensor initialised successfully");
  return true;
}
