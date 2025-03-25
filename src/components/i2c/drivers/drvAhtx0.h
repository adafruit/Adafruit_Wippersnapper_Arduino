/*!
 * @file drvAhtx0.h
 *
 * Device driver for an AHT Humidity and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_AHTX0_H
#define DRV_AHTX0_H

#include "drvBase.h"
#include <Adafruit_AHTX0.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the AHTX0 temperature
            and humidity sensor.
*/
/**************************************************************************/
class drvAhtx0 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an AHTX0 sensor.
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
  drvAhtx0(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AHTX0 sensor.
  */
  /*******************************************************************************/
  ~drvAhtx0() { delete _aht; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AHTX0 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() override {
    // attempt to initialize the driver
    _aht = new Adafruit_AHTX0();
    if (!_aht->begin(_i2c, (int32_t)_address))
      return false;

    // initialize sensors
    _aht_temp = _aht->getTemperatureSensor();
    if (_aht_temp == NULL)
      return false;
    _aht_humidity = _aht->getHumiditySensor();
    if (_aht_humidity == NULL)
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    _aht_temp->getEvent(tempEvent);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    _aht_humidity->getEvent(humidEvent);
    return true;
  }

protected:
  Adafruit_AHTX0 *_aht; ///< Pointer to an AHTX0 object
  Adafruit_Sensor *_aht_temp =
      NULL; ///< Holds data for the AHTX0's temperature sensor
  Adafruit_Sensor *_aht_humidity =
      NULL; ///< Holds data for the AHTX0's humidity sensor
  wippersnapper_sensor_SensorType _default_sensor_types[3] = {
      wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE,
      wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT,
      wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY}; ///< Default
                                                                      ///< sensor
                                                                      ///< types
};

#endif // drvAhtx0