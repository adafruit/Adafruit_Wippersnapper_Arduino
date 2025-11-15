/*!
 * @file drvLsm6dso32.h
 *
 * Driver wrapper for the Adafruit LSM6DSO32 6-DoF IMU.
 */
#ifndef DRV_LSM6DSO32_H
#define DRV_LSM6DSO32_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"

#include <Adafruit_LSM6DSO32.h>

class drvLsm6dso32 : public drvBase {
public:
  drvLsm6dso32(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  ~drvLsm6dso32();

  bool begin() override;

  bool getEventRaw(sensors_event_t *rawEvent) override;

  bool getEventBoolean(sensors_event_t *booleanEvent) override;

  bool getEventAccelerometer(sensors_event_t *accelEvent) override;

  bool getEventGyroscope(sensors_event_t *gyroEvent) override;

protected:
  void ConfigureDefaultSensorTypes() override;

private:
  bool readAllEvents(sensors_event_t *accel, sensors_event_t *gyro,
                     sensors_event_t *temp);
  bool computeAccelMagnitude(float &magnitude);

  Adafruit_LSM6DSO32 *_imu = nullptr;
};

#endif // DRV_LSM6DSO32_H
