/*!
 * @file drvLsm6ds3.h
 *
 * Driver wrapper for the Adafruit LSM6DS3 and LSM6DS3TR-C 6-DoF IMUs.
 */
#ifndef DRV_LSM6DS3_H
#define DRV_LSM6DS3_H

#include "Wippersnapper_V2.h"
#include "drvBaseAccelLsm6.h"

#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LSM6DS3TRC.h>

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
  // Adafruit_LSM6DS3 (WHO_AM_I 0x69) or Adafruit_LSM6DS3TRC (0x6A)
  Adafruit_LSM6DS *_imu = nullptr;
};

#endif // DRV_LSM6DS3_H
