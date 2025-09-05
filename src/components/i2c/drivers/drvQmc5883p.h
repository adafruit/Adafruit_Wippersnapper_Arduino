/*!
 * @file drvQmc5883p.h
 *
 * Driver wrapper for the Adafruit QMC5883P 3-axis magnetometer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_QMC5883P_H
#define DRV_QMC5883P_H

#include "drvBase.h"

class Adafruit_QMC5883P;  // forward

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a QMC5883P sensor.
*/
/**************************************************************************/
class drvQmc5883p : public drvBase {
 public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a QMC5883P sensor.
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
  drvQmc5883p(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for a QMC5883P sensor.
  */
  /*******************************************************************************/
  ~drvQmc5883p();

  /*******************************************************************************/
  /*!
      @brief    Initializes the QMC5883P sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin();

  /*******************************************************************************/
  /*!
      @brief    Gets the QMC5883P's magnetometer sensor event.
      @param    magEvent
                Pointer to the magnetometer sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *magEvent);

  /*******************************************************************************/
  /*!
      @brief    Gets the QMC5883P's magnetic field vector.
      @param    magneticEvent
                Pointer to the magnetic field sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventMagneticField(sensors_event_t *magneticEvent);

  void ConfigureDefaultSensorTypes() override;

 protected:
  Adafruit_QMC5883P *_qmc = nullptr;  ///< Pointer to QMC5883P sensor object
};

#endif  // DRV_QMC5883P_H