/*!
 * @file drvHts221.h
 *
 * Device driver for the HTS221 Humidity and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2025 Tyeth Gundry for Adafruit Industries
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_HTS221_H
#define DRV_HTS221_H

#include "drvBase.h"
#include <Adafruit_HTS221.h>

/*!
    @brief  Class that provides a sensor driver for the HTS221 humidity and
            temperature sensor. This implementation uses the 1 Hz data rate.
*/
class drvHts221 : public drvBase {

public:
  /*!
      @brief    Constructor for an HTS221 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvHts221(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an HTS221 sensor.
  */
  ~drvHts221() { delete _hts221; }

  /*!
      @brief    Initializes the HTS221 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  bool begin() override {
    // attempt to initialize the HTS221 using the I2C interface
    _hts221 = new Adafruit_HTS221();
    if (!_hts221->begin_I2C(_address, _i2c))
      return false;

    // set the HTS221's data rate to 1 Hz
    _hts221->setDataRate(HTS221_RATE_1_HZ);

    // get temperature and humidity sensor
    _hts221_temp = _hts221->getTemperatureSensor();
    if (_hts221_temp == NULL)
      return false;

    _hts221_humidity = _hts221->getHumiditySensor();
    if (_hts221_humidity == NULL)
      return false;

    return true;
  }

  /*!
      @brief    Gets the HTS221's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _hts221_temp->getEvent(tempEvent);
  }

  /*!
      @brief    Gets the HTS221's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    return _hts221_humidity->getEvent(humidEvent);
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
  }

protected:
  Adafruit_HTS221 *_hts221; ///< Pointer to an HTS221 object
  Adafruit_Sensor *_hts221_temp =
      NULL; ///< Holds data for the HTS221's temperature sensor
  Adafruit_Sensor *_hts221_humidity =
      NULL; ///< Holds data for the HTS221's humidity sensor
};

#endif // drvHts221