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
  void setInternalPollingInterval(uint32_t interval_ms) { // override {
    // Polling interval is managed internally.
    _internalPollPeriod = interval_ms < 0 ? 0 : interval_ms;
  }

private:
  bool readAllEvents(sensors_event_t *accel, sensors_event_t *gyro,
                     sensors_event_t *temp, boolean *shake,
                     uint16_t *steps);
  bool readAllEvents();
  bool computeAccelMagnitude(float &magnitude);

  bool _has_last_events = false;   ///< Flag to track if last events are stored
  bool _last_shake = false;        ///< Last state of shake / tap detection
  sensors_event_t _lastAccelEvent; ///< Last accelerometer event
  sensors_event_t _lastGyroEvent;  ///< Last gyroscope event
  sensors_event_t _lastTempEvent;  ///< Last temperature event (raw)
  uint16_t _last_steps = 0;        ///< Last step count
  uint32_t _lastPoll = 0;          ///< Last poll time
  uint32_t _internalPollPeriod = 200; ///< Internal Polling interval in ms

  Adafruit_ISM330DHCX *_imu = nullptr; ///< Pointer to the ISM330DHCX sensor
};

#endif // DRV_ISM330DHCX_H
