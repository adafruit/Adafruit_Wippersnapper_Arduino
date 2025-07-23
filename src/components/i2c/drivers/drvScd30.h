/*!
 * @file drvScd30.h
 *
 * Device driver for the SCD30 CO2, Temperature, and Humidity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SCD30_H
#define DRV_SCD30_H

#include "drvBase.h"
#include <Adafruit_SCD30.h>

/*!
    @brief  Class that provides a driver interface for the SCD30 sensor.
*/
class drvScd30 : public drvBase {

public:
  /*!
      @brief    Constructor for a SCD30 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvScd30(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the SCD30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _scd = new Adafruit_SCD30();
    return _scd->begin((uint8_t)_address, _i2c);
  }

  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  bool hasBeenReadInLastSecond() {
    return _lastRead != 0 && millis() - _lastRead < 1000;
  }

  /*!
      @brief    Checks if the sensor is ready to be read
      @returns  True if the sensor is ready, False otherwise.
  */
  bool isSensorReady() {
    if (!_scd->dataReady()) {
      // failed, one more quick attempt
      delay(100);
      if (!_scd->dataReady()) {
        return false;
      }
    }
    return true;
  }

  /*!
      @brief    Reads the SCD30 sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  bool readSensorData() {
    // dont read sensor more than once per second
    if (hasBeenReadInLastSecond()) {
      return true;
    }

    if (!isSensorReady()) {
      return false;
    }

    if (!_scd->read()) {
      return false;
    }
    _CO2 = _scd->CO2;
    _humidity = _scd->relative_humidity;
    _temperature = _scd->temperature;
    _lastRead = millis();
    return true;
  }

  /*!
      @brief    Gets the SCD30's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // check if sensor is enabled and data is available
    if (!readSensorData()) {
      return false;
    }

    tempEvent->temperature = _temperature;
    return true;
  }

  /*!
      @brief    Gets the SCD30's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // check if sensor is enabled and data is available
    if (!readSensorData()) {
      return false;
    }

    humidEvent->relative_humidity = _humidity;
    return true;
  }

  /*!
      @brief    Gets the SCD30's current CO2 reading.
      @param    co2Event
                  Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventCO2(sensors_event_t *co2Event) {
    // check if sensor is enabled and data is available
    if (!readSensorData()) {
      return false;
    }

    co2Event->CO2 = _CO2;
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 4;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
    _default_sensor_types[3] = wippersnapper_sensor_SensorType_SENSOR_TYPE_CO2;
  }

protected:
  Adafruit_SCD30 *_scd = nullptr; ///< SCD30 driver object
  ulong _lastRead = 0;            ///< Last time the sensor was read
  float _temperature;             ///< Temperature
  float _humidity;                ///< Relative Humidity
  float _CO2;                     ///< CO2
};

#endif // drvScd30