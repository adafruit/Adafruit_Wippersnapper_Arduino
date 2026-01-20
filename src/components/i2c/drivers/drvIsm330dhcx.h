/*!
 * @file drvIsm330dhcx.h
 *
 * Driver wrapper for the Adafruit ISM330DHCX (LSM6DSOX core) 6-DoF IMU.
 */
#ifndef DRV_ISM330DHCX_H
#define DRV_ISM330DHCX_H

#include "Wippersnapper_V2.h"
#include "drvBaseAccelLsm6.h"
#include <Adafruit_ISM330DHCX.h>

#define ISM330_TAP_THRESHOLD_MSS 15.0f

class drvIsm330dhcx : public drvBaseAccelLsm6 {
public:
  drvIsm330dhcx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name);
  ~drvIsm330dhcx();

  bool begin() override;

protected:
  Adafruit_LSM6DS *getLSM6Sensor() const override { return _imu; }

private:
  Adafruit_ISM330DHCX *_imu = nullptr; ///< Pointer to the ISM330DHCX sensor
};

#endif // DRV_ISM330DHCX_H
