/*!
 * @file drvIna238.h
 *
 * Device driver for the INA238 High Precision DC Current and Voltage Monitor
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
#ifndef DRV_INA238_H
#define DRV_INA238_H

#include "drvBase.h"

class Adafruit_INA238;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA238 sensor.
*/
/**************************************************************************/
class drvIna238 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA238 sensor.
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
  drvIna238(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA238 sensor.
  */
  /*******************************************************************************/
  ~drvIna238();

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA238 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin();

  /*******************************************************************************/
  /*!
      @brief    Reads a voltage sensor and converts the
                reading into the expected SI unit.
      @param    voltageEvent
                voltage sensor reading, in volts.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVoltage(sensors_event_t *voltageEvent);

  /**
   * @brief   Get the current sensor event.
   *
   * @param   currentEvent  Pointer to the current sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventCurrent(sensors_event_t *currentEvent);

  void ConfigureDefaultSensorTypes() override;

protected:
  Adafruit_INA238 *_ina238; ///< Pointer to INA238 sensor object
};

#endif // DRV_INA238_H