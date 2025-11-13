/*!
 * @file drvLis3dh.h
 *
 * Driver wrapper for the Adafruit LIS3DH 3-axis accelerometer.
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
#ifndef DRV_LIS3DH_H
#define DRV_LIS3DH_H

#include "drvBase.h"
#include <Adafruit_LIS3DH.h>

class Adafruit_LIS3DH; // forward

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LIS3DH accelerometer.
*/
/**************************************************************************/
class drvLis3dh : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LIS3DH sensor.
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
  drvLis3dh(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for a LIS3DH sensor.
  */
  /*******************************************************************************/
  ~drvLis3dh();

  /*******************************************************************************/
  /*!
      @brief    Initializes the LIS3DH sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override;

  /*******************************************************************************/
  /*!
      @brief    Gets the LIS3DH's raw sensor event (magnitude stored in
                event->data[0]).
      @param    rawEvent
                Pointer to the sensor event to fill.
      @returns  True if the event was obtained successfully, False otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) override;

  /*******************************************************************************/
  /*!
      @brief    Gets the LIS3DH's accelerometer sensor event (x,y,z in m/s^2).
      @param    accelEvent
                Pointer to the accelerometer sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAccelerometer(sensors_event_t *accelEvent) override;

  void ConfigureDefaultSensorTypes() override;

protected:
  Adafruit_LIS3DH *_lis = nullptr; ///< Pointer to LIS3DH sensor object
};

#endif // DRV_LIS3DH_H