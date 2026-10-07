/*!
 * @file drvBaseAccelLsm6.h
 *
 * Shared helper for Adafruit LSM6-series accelerometer/gyroscope drivers
 * that expose shake and pedometer functionality.
 */
#ifndef DRV_BASE_ACCEL_LSM6_H
#define DRV_BASE_ACCEL_LSM6_H

#include "drvBase.h"

#include <Adafruit_LSM6DS.h>

class drvBaseAccelLsm6 : public drvBase {
public:
  drvBaseAccelLsm6(TwoWire *i2c, uint16_t sensorAddress,
                   uint32_t mux_channel, const char *driver_name);
  virtual ~drvBaseAccelLsm6();

  bool getEventBoolean(sensors_event_t *booleanEvent) override;
  bool getEventRaw(sensors_event_t *rawEvent) override;
  bool getEventAccelerometer(sensors_event_t *accelEvent) override;
  bool getEventGyroscope(sensors_event_t *gyroEvent) override;
  bool getEventAmbientTemp(sensors_event_t *temperatureEvent) override;

  void ConfigureDefaultSensorTypes() override;

  void setInternalPollingInterval(uint32_t interval_ms);

protected:
  virtual Adafruit_LSM6DS *getLSM6Sensor() const = 0;
  
  /*!
      @brief    Builds a unique Adafruit_Sensor ID for this IMU from its I2C
                address, MUX address/channel (if set) and I2C bus (default
                or alt.), so several LSM6 parts can run at once. Passed to
                begin_I2C(), which uses id, id + 1 and id + 2 for the accel,
                gyro and temp events.
      @returns  The base sensor ID for this driver instance.
  */
  uint32_t getLsmSensorID() {
    // allow 4 ids per sensor (acc/mag/gyro/temp)
    uint32_t sensor_id = 10 * GetAddress();
    if (GetMuxAddress() != 0x0) {
      sensor_id += 10000 + GetMuxAddress();
      sensor_id += 1000 + (1000 * GetMuxChannel());
    }
    // alt. bus offset sits above the address + MUX range (max 19389)
    if (HasAltI2CBus()) {
      sensor_id += 100000;
    }
    return sensor_id;
  }

  bool readAllEvents();
  bool computeAccelMagnitude(float *magnitude);

  bool _has_last_events = false;   ///< Flag to track if last events are stored
  bool _last_shake = false;        ///< Last state of shake / tap detection
  sensors_event_t _lastAccelEvent; ///< Last accelerometer event
  sensors_event_t _lastGyroEvent;  ///< Last gyroscope event
  sensors_event_t _lastTempEvent;  ///< Last temperature event (raw)
//   uint16_t _last_steps = 0;        ///< Last step count
  uint32_t _lastPoll = 0;          ///< Last poll time
  uint32_t _internalPollPeriod = 200; ///< Internal Polling interval in ms
};

#endif // DRV_BASE_ACCEL_LSM6_H
