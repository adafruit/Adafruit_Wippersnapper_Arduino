/*!
 * @file drvIsm330dhcx.h
 *
 * Driver wrapper for the Adafruit ISM330DHCX (LSM6DSOX core) 6-DoF IMU.
 */
#ifndef DRV_ISM330DHCX_H
#define DRV_ISM330DHCX_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"
#include <Adafruit_ISM330DHCX.h>

#define ISM330_TAP_THRESHOLD_MSS 15.0f

class drvIsm330dhcx : public drvBase {
public:
  drvIsm330dhcx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name);
  ~drvIsm330dhcx();

  bool begin() override;
  bool getEventBoolean(sensors_event_t *booleanEvent) override;
  bool getEventRaw(sensors_event_t *rawEvent) override;
  bool getEventAccelerometer(sensors_event_t *accelEvent) override;
  bool getEventGyroscope(sensors_event_t *gyroEvent) override;

  void ConfigureDefaultSensorTypes() override;

private:
  bool readAllEvents(sensors_event_t *accel, sensors_event_t *gyro,
                     sensors_event_t *temp);
  bool computeAccelMagnitude(float &magnitude);

  Adafruit_ISM330DHCX *_imu = nullptr; ///< Pointer to the ISM330DHCX sensor
};

#endif // DRV_ISM330DHCX_H
