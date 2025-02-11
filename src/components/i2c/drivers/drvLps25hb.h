/*!
 * @file drvLps25hb.h
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

#ifndef DRV_LPS25HB_H
#define DRV_LPS25HB_H

#include "drvBase.h"
#include <Adafruit_LPS2X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS25HB temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvLps25hb : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS25HB sensor.
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
  drvLps25hb(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS25HB sensor.
  */
  /*******************************************************************************/
  ~drvLps25hb() { delete _lps25; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS25HB sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _lps25 = new Adafruit_LPS25();
    // attempt to initialize LPS25HB
    if (!_lps25->begin_I2C(_address, _i2c))
      return false;

    // Set up sample rate and filter initialization
    _lps25->setDataRate(LPS25_RATE_ONE_SHOT);
    _temp = _lps25->getTemperatureSensor();
    if (_temp == NULL)
      return false;
    _pressure = _lps25->getPressureSensor();
    if (_pressure == NULL)
      return false;
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
    return _temp->getEvent(tempEvent);
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
    return _pressure->getEvent(pressureEvent);
  }

protected:
  Adafruit_LPS25 *_lps25; ///< LPS25HB  object
  Adafruit_Sensor *_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
};

#endif // drvLps25hb