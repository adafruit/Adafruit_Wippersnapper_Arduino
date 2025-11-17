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
  Adafruit_LSM6DSO32 *_imu = nullptr;
};

#endif // DRV_LSM6DSO32_H
