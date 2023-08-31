/*!
 * @file WipperSnapper_I2C_Driver_MS8607.h
 *
 * Device driver for an MS8607 Pressure Humidity and Temperature sensor.
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

#ifndef WipperSnapper_I2C_Driver_MS8607_H
#define WipperSnapper_I2C_Driver_MS8607_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MS8607.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the MS8607 PHT sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_MS8607 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MS8607 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_MS8607(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MS8607 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_MS8607() { delete _ms8607; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MS8607 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ms8607 = new Adafruit_MS8607();
    // attempt to initialize MS8607
    if (!_ms8607->begin(_i2c))
      return false;

    _ms8607_temp = _ms8607->getTemperatureSensor();
    _ms8607_humidity = _ms8607->getHumiditySensor();
    _ms8607_pressure = _ms8607->getPressureSensor();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MS8607's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (_ms8607_temp == NULL)
      return false;
    _ms8607_temp->getEvent(tempEvent);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MS8607's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (_ms8607_humidity == NULL)
      return false;
    _ms8607_humidity->getEvent(humidEvent);
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
    if (_ms8607_pressure == NULL)
      return false;
    _ms8607_pressure->getEvent(pressureEvent);
    return true;
  }

protected:
  Adafruit_MS8607 *_ms8607; ///< MS8607  object
  Adafruit_Sensor *_ms8607_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_ms8607_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
  Adafruit_Sensor *_ms8607_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
};

#endif // WipperSnapper_I2C_Driver_MS8607