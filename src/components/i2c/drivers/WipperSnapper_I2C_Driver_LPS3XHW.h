/*!
 * @file WipperSnapper_I2C_Driver_LPS3XHW.h
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

#ifndef WipperSnapper_I2C_Driver_LPS3XHW_H
#define WipperSnapper_I2C_Driver_LPS3XHW_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LPS35HW.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS3XHW temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LPS3XHW : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS3XHW sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LPS3XHW(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS3XHW sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LPS3XHW() { delete _lps3xhw; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS3XHW sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _lps3xhw = new Adafruit_LPS35HW();
    // attempt to initialize LPS3XHW
    if (!_lps3xhw->begin_I2C(_sensorAddress, _i2c))
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

#endif // WipperSnapper_I2C_Driver_LPS3XHW