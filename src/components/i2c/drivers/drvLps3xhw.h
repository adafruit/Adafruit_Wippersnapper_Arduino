/*!
 * @file drvLps3xhw.h
 *
 * Device driver for a LPS3XHW precision pressure sensor breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_LPS3XHW_H
#define DRV_LPS3XHW_H

#include "drvBase.h"
#include <Adafruit_LPS35HW.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS3XHW temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvLps3xhw : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS3XHW sensor.
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
  drvLps3xhw(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS3XHW sensor.
  */
  /*******************************************************************************/
  ~drvLps3xhw() { delete _lps3xhw; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS3XHW sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _lps3xhw = new Adafruit_LPS35HW();
    // attempt to initialize LPS3XHW
    if (!_lps3xhw->begin_I2C(_address, _i2c))
      return false;

    // Set up sample rate and filter initialization
    _lps3xhw->setDataRate(LPS35HW_RATE_ONE_SHOT);
    _lps3xhw->enableLowPass();

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the LPS3XHW's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    _lps3xhw->takeMeasurement();
    tempEvent->temperature = _lps3xhw->readTemperature();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    _lps3xhw->takeMeasurement();
    pressureEvent->pressure = _lps3xhw->readPressure();
    return true;
  }

protected:
  Adafruit_LPS35HW *_lps3xhw; ///< LPS3XHW  object
};

#endif // drvLps3xhw