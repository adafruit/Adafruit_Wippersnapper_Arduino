/*!
 * @file drvBme280.h
 *
 * Device driver for a BME280 Pressure Humidity and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_BME_280_H
#define DRV_BME_280_H

#include "drvBase.h"
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BME280 temperature
            and humidity sensor.
*/
/**************************************************************************/
class drvBme280 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BME280 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C MUX channel, if applicable.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvBme280(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BME280 sensor.
  */
  /*******************************************************************************/
  ~drvBme280() { delete _bme; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BME280 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _bme = new Adafruit_BME280();
    // attempt to initialize BME280
    if (!_bme->begin(_address, _i2c))
      return false;

    // Configure sensors
    _bme_temp = _bme->getTemperatureSensor();
    if (_bme_temp == NULL)
      return false;

    _bme_humidity = _bme->getHumiditySensor();
    if (_bme_humidity == NULL)
      return false;

    _bme_pressure = _bme->getPressureSensor();
    if (_bme_pressure == NULL)
      return false;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BME280's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _bme_temp->getEvent(tempEvent);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BME280's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    return _bme_humidity->getEvent(humidEvent);
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
    return _bme_pressure->getEvent(pressureEvent);
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a the BME280's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    altitudeEvent->altitude = _bme->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 5;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
    _default_sensor_types[3] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE;
    _default_sensor_types[4] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_ALTITUDE;
  }

protected:
  Adafruit_BME280 *_bme; ///< BME280  object
  Adafruit_Sensor *_bme_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_bme_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
  Adafruit_Sensor *_bme_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
};

#endif // drvBme280