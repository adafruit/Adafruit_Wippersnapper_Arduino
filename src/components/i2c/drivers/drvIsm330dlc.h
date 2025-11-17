/*!
 * @file drvIsm330dlc.h
 *
 * Driver wrapper for the Adafruit ISM330DLC (LSM6DSL core) 6-DoF IMU.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * MIT license, all text here must be included in any redistribution.
 */
#ifndef DRV_ISM330DLC_H
#define DRV_ISM330DLC_H

#include "Wippersnapper_V2.h"
#include "drvBaseAccelLsm6.h"
#include <Adafruit_LSM6DSL.h>

#define ISM330_TAP_THRESHOLD_MSS 15.0f

class drvIsm330dlc : public drvBaseAccelLsm6 {
public:
  drvIsm330dlc(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name);
  ~drvIsm330dlc();

  bool begin() override;

protected:
  Adafruit_LSM6DS *getLSM6Sensor() const override { return _imu; }

private:
  Adafruit_LSM6DSL *_imu = nullptr; ///< Pointer to the ISM330DLC sensor
};

#endif // DRV_ISM330DLC_H
