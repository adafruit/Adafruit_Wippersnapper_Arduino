/*!
 * @file drvLsm6ds3.h
 *
 * Driver wrapper for the Adafruit LSM6DS3 6-DoF IMU.
 */
#ifndef DRV_LSM6DS3_H
#define DRV_LSM6DS3_H

#include "Wippersnapper_V2.h"
#include "drvBaseAccelLsm6.h"

#include <Adafruit_LSM6DS3.h>

class drvLsm6ds3 : public drvBaseAccelLsm6 {
public:
    drvLsm6ds3(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
          const char *driver_name)
        : drvBaseAccelLsm6(i2c, sensorAddress, mux_channel, driver_name) {}

  ~drvLsm6ds3();

  bool begin() override;

protected:
  Adafruit_LSM6DS *getLSM6Sensor() const override { return _imu; }

private:
  Adafruit_LSM6DS3 *_imu = nullptr;
};

#endif // DRV_LSM6DS3_H
