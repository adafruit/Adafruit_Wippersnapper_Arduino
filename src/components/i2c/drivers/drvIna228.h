/*!
 * @file drvIna228.h
 *
 * Device driver for the INA228 High Precision DC Current and Voltage Monitor
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
#ifndef DRV_INA228_H
#define DRV_INA228_H

#include "drvBase.h"

class Adafruit_INA228;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA228 sensor.
*/
/**************************************************************************/
class drvIna228 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA228 sensor.
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
  drvIna228(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA228 sensor.
  */
  /*******************************************************************************/
  ~drvIna228();

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA228 sensor and begins I2C.
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
  Adafruit_INA228 *_ina228 = nullptr; ///< Pointer to INA228 sensor object
};

#endif // DRV_INA228_H