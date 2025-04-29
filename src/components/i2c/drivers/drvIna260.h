/*!
 * @file drvIna260.h
 *
 * Device driver for the INA260 DC Current and Voltage Monitor
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
#ifndef DRV_INA260_H
#define DRV_INA260_H

#include "drvBase.h"
#include <Adafruit_INA260.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA260 sensor.
*/
/**************************************************************************/
class drvIna260 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA260 sensor.
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
  drvIna260(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA260 sensor.
  */
  /*******************************************************************************/
  ~drvIna260() { delete _ina260; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA260 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ina260 = new Adafruit_INA260();
    if (!_ina260->begin(_address, _i2c)) {
      WS_DEBUG_PRINTLN("INA260 failed to initialise!");
      return false;
    }
    // TODO: use setCalibration()

    return true;
  }

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
  bool getEventVoltage(sensors_event_t *voltageEvent) {
    voltageEvent->voltage = _ina260->readBusVoltage();
    return true;
  }

  /**
   * @brief   Get the current sensor event.
   *
   * @param   currentEvent  Pointer to the current sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventCurrent(sensors_event_t *currentEvent) {
    currentEvent->current = _ina260->readCurrent();
    return true;
  }

protected:
  Adafruit_INA260 *_ina260; ///< Pointer to INA260 sensor object
};

#endif // DRV_INA260_H