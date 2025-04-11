/*!
 * @file drvSht3x.h
 *
 * Device driver for the SHT3X Temperature and Humidity Sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Marni Brewster 2022 for Adafruit Industries.
 * Copyright (c) Tyeth Gundry 2022. Original code by Marni,
 * rewritten to use driver by Sensirion, help from Brent Rubell.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SHT3X_H
#define DRV_SHT3X_H

#include "drvBase.h"
#include <SHTSensor.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SHT3X sensor.
*/
/**************************************************************************/
class drvSht3x : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SHT3X sensor.
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
  drvSht3x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SHT3X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_address == 0x44) // if address 0x44 (dec:68), alternative = 0x45
      _sht3x = new SHTSensor(SHTSensor::SHT3X);
    else
      _sht3x = new SHTSensor(SHTSensor::SHT3X_ALT);

    if (!_sht3x->init(*_i2c))
      return false;

    // Use HIGH PRECISION - only supported by 3X/4X
    return _sht3x->setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHT3X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // populate temp and humidity objects with fresh data
    if (!_sht3x->readSample())
      return false;
    tempEvent->temperature = _sht3x->getTemperature();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHT3X's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // populate temp and humidity objects with fresh data
    if (!_sht3x->readSample())
      return false;
    humidEvent->relative_humidity = _sht3x->getHumidity();
    return true;
  }

protected:
  SHTSensor *_sht3x; ///< SHT3X object
};

#endif // drvSht3x