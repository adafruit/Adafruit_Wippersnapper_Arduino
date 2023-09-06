/*!
 * @file WipperSnapper_I2C_Driver_LPS22HB.h
 *
 * Device driver for a LPS22HB precision pressure sensor breakout.
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

#ifndef WipperSnapper_I2C_Driver_LPS22HB_H
#define WipperSnapper_I2C_Driver_LPS22HB_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LPS2X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS22HB temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LPS22HB : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS22HB sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LPS22HB(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS22HB sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LPS22HB() { delete _lps22; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS22HB sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _lps22 = new Adafruit_LPS22();
    // attempt to initialize LPS22HB
    if (!_lps22->begin_I2C(_sensorAddress, _i2c))
      return false;

    // Set up sample rate and filter initialization
    _lps22->setDataRate(LPS22_RATE_ONE_SHOT);
    _temp = _lps22->getTemperatureSensor();
    _pressure = _lps22->getPressureSensor();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the LPS22HB's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (_temp == NULL)
      return false;
    _temp->getEvent(tempEvent);
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
    if (_pressure == NULL)
      return false;
    _pressure->getEvent(pressureEvent);
    return true;
  }

protected:
  Adafruit_LPS22 *_lps22; ///< LPS22HB  object
  Adafruit_Sensor *_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
};

#endif // WipperSnapper_I2C_Driver_LPS22HB