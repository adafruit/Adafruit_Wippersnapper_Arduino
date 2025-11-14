/*!
 * @file drvLsm303dlh.h
 *
 * Driver wrapper for the classic Adafruit LSM303DLH combo sensor.
 */
#ifndef DRV_LSM303DLH_H
#define DRV_LSM303DLH_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"

#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>

class drvLsm303dlh : public drvBase {
public:
  drvLsm303dlh(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  ~drvLsm303dlh();

  bool begin() override;

  bool getEventRaw(sensors_event_t *rawEvent) override;

  bool getEventAccelerometer(sensors_event_t *accelEvent) override;

  bool getEventMagneticField(sensors_event_t *magEvent) override;

protected:
  void ConfigureDefaultSensorTypes() override;

private:
  void teardown();
  bool computeAccelMagnitude(float &magnitude);

  Adafruit_LSM303_Accel_Unified *_accel = nullptr;
  Adafruit_LSM303DLH_Mag_Unified *_mag = nullptr;
};

#endif // DRV_LSM303DLH_H
