/*!
 * @file drvLsm6dso32.h
 *
 * Driver wrapper for the Adafruit LSM6DSO32 6-DoF IMU.
 */
#ifndef DRV_LSM6DSO32_H
#define DRV_LSM6DSO32_H

#include "Wippersnapper_V2.h"
#include "drvBaseAccelLsm6.h"

#include <Adafruit_LSM6DSO32.h>

class drvLsm6dso32 : public drvBaseAccelLsm6 {
public:
  drvLsm6dso32(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name)
      : drvBaseAccelLsm6(i2c, sensorAddress, mux_channel, driver_name) {}

  ~drvLsm6dso32();

  bool begin() override;

protected:
  Adafruit_LSM6DS *getLSM6Sensor() const override { return _imu; }

private:
  bool isDsoxScale();

  // Adafruit_LSM6DSO32, or Adafruit_LSM6DSOX for an LSM6DSOX (same
  // WHO_AM_I 0x6C, but half the accel full-scale for each range code)
  Adafruit_LSM6DSOX *_imu = nullptr;
};

#endif // DRV_LSM6DSO32_H
