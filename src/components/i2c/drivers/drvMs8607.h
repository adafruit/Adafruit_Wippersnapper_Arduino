/*!
 * @file drvMs8607.h
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

#ifndef DRV_MS8607
#define DRV_MS8607

#include "drvBase.h"
#include <Adafruit_MS8607.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the MS8607 PHT sensor.
*/
/**************************************************************************/
class drvMs8607 : public drvBase {

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
  drvMs8607(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel, const char* driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an MS8607 sensor.
  */
  /*******************************************************************************/
  ~drvMs8607() { delete _ms8607; }

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
    if (_ms8607_temp == NULL)
      return false;
    _ms8607_humidity = _ms8607->getHumiditySensor();
    if (_ms8607_humidity == NULL)
      return false;
    _ms8607_pressure = _ms8607->getPressureSensor();
    if (_ms8607_pressure == NULL)
      return false;

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

#endif // drvMs8607