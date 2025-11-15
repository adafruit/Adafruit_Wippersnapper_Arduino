/*!
 * @file drvLsm9ds1.h
 *
 * Driver wrapper for the Adafruit LSM9DS1 9-DOF IMU.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_LSM9DS1_H
#define DRV_LSM9DS1_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"
#include <Adafruit_LSM9DS1.h>

class Adafruit_LSM9DS1; // forward

// Approximate acceleration magnitude (m/s^2) that triggers a tap event
#define LSM9DS1_TAP_THRESHOLD_MSS 15.0f

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LSM9DS1 IMU.
*/
/**************************************************************************/
class drvLsm9ds1 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LSM9DS1 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvLsm9ds1(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for a LSM9DS1 sensor.
  */
  /*******************************************************************************/
  ~drvLsm9ds1();

  /*******************************************************************************/
  /*!
      @brief    Initializes the LSM9DS1 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override;

  /******************************************************************************/
  /*!
      @brief    Gets the LSM9DS1's boolean sensor event.
      @param    booleanEvent
                Pointer to the sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /******************************************************************************/
  bool getEventBoolean(sensors_event_t *booleanEvent) override;

  /*******************************************************************************/
  /*!
      @brief    Gets the LSM9DS1's raw sensor event (magnitude stored in
                event->data[0]).
      @param    rawEvent
                Pointer to the sensor event to fill.
      @returns  True if the event was obtained successfully, False otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) override;

  /*******************************************************************************/
  /*!
      @brief    Gets the LSM9DS1's accelerometer sensor event (x,y,z in m/s^2).
      @param    accelEvent
                Pointer to the accelerometer sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAccelerometer(sensors_event_t *accelEvent) override;

  /*******************************************************************************/
  /*!
      @brief    Gets the LSM9DS1's gyroscope sensor event (x,y,z in rad/s).
      @param    gyroEvent
                Pointer to the gyroscope sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventGyroscope(sensors_event_t *gyroEvent) override;

  /*******************************************************************************/
  /*!
      @brief    Gets the LSM9DS1's magnetometer sensor event (x,y,z in uT).
      @param    magEvent
                Pointer to the magnetometer sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventMagneticField(sensors_event_t *magEvent) override;

  void ConfigureDefaultSensorTypes() override;

protected:
  Adafruit_LSM9DS1 *_lsm = nullptr; ///< Pointer to LSM9DS1 sensor object

  bool readAllEvents(sensors_event_t *accel, sensors_event_t *mag,
                     sensors_event_t *gyro, sensors_event_t *temp);
};

#endif // DRV_LSM9DS1_H
