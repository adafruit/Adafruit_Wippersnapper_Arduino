/*!
 * @file drvLps28dfw.h
 *
 * Device driver for a LPS28DFW precision pressure sensor breakout.
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

#ifndef DRV_LPS28DFW_H
#define DRV_LPS28DFW_H

#include "drvBase.h"
#include <Adafruit_LPS28.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the LPS28DFW temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvLps28dfw : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an LPS28DFW sensor.
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
  drvLps28dfw(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LPS28DFW sensor.
  */
  /*******************************************************************************/
  ~drvLps28dfw() { delete _lps28; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LPS28DFW sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _lps28 = new Adafruit_LPS28();
    // attempt to initialize LPS28DFW
    if (!_lps28->begin(_i2c, _address))
      return false;

    // Set up sample rate and filter initialization
    if (!_lps28->setDataRate(LPS28_ODR_ONESHOT)) {
      WS_DEBUG_PRINTLN("Failed to set data rate");
      return false;
    }
    if (!_lps28->setAveraging(LPS28_AVG_512)) {
      WS_DEBUG_PRINTLN("Failed to set averaging");
      return false;
    }
    if (!_lps28->setFullScaleMode(true)) {
      WS_DEBUG_PRINTLN("Failed to set 4060hPa max mode");
      return false;
    }

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the sensor and stores the data in the object.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool readSensor() {
    // grab one reading to seed the sensor
    if (!_lps28->triggerOneShot()) {
      return false;
    }

    // Wait (block up to 100ms) until data is ready
    for (uint8_t i = 0; i < 100; i++) {
      if (_lps28->getStatus() & LPS28_STATUS_PRESS_READY) {
        if (_temp == NULL) {
          _temp = _lps28->getTemperatureSensor();
        }
        if (_pressure == NULL) {
          _pressure = _lps28->getPressureSensor();
        }
        return true;
      }
      delay(1);
    }
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the LPS28DFW's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!readSensor())
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
    if (!readSensor())
      return false;
    _pressure->getEvent(pressureEvent);
    return true;
  }

protected:
  Adafruit_LPS28 *_lps28 = nullptr; ///< LPS28DFW  object
  Adafruit_Sensor *_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
};

#endif // DRV_LPS28DFW_H