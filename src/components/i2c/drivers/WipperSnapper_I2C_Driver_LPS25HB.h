/*!
 * @file WipperSnapper_I2C_Driver_LPS25HB.h
 *
 * Device driver for a LPS25HB precision pressure sensor breakout.
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

#ifndef WipperSnapper_I2C_Driver_LPS25HB_H
#define WipperSnapper_I2C_Driver_LPS25HB_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_LPS2X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS25HB temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_LPS25HB : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS25HB sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_LPS25HB(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS25HB sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_LPS25HB() { delete _lps25; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS25HB sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _lps25 = new Adafruit_LPS25();
    // attempt to initialize LPS25HB
    if (!_lps25->begin_I2C(_sensorAddress, _i2c))
      return false;

    // Set up sample rate and filter initialization
    _lps25->setDataRate(LPS25_RATE_ONE_SHOT);
    _temp = _lps25->getTemperatureSensor();
    _pressure = _lps25->getPressureSensor();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the LPS25HB's current temperature.
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
  Adafruit_LPS25 *_lps25; ///< LPS25HB  object
  Adafruit_Sensor *_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
};

#endif // WipperSnapper_I2C_Driver_LPS25HB