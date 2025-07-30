/*!
 * @file drvPm25.h
 *
 * I2C driver for the Adafruit PM2.5 Air Quality Sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022-2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_PM25_H
#define DRV_PM25_H

#include "drvBase.h"
#include <Adafruit_PM25AQI.h>
#include <Wire.h>

/*!
    @brief  Class that provides a driver interface for the PM25 sensor.
*/
class drvPm25 : public drvBase {

public:
  /*!
      @brief    Constructor for a PM25 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvPm25(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
          const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the PM25 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _pm25 = new Adafruit_PM25AQI();
    // Wait three seconds for the sensor to boot up!
    delay(3000);
    return _pm25->begin_I2C(_i2c);
  }

  /*!
      @brief    Gets the PM25 sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    PM25_AQI_Data data;
    if (!_pm25->read(&data)) {
      WS_DEBUG_PRINTLN("Failed to read PM10STD data");
      return false; // couldn't read data
    }

    pm10StdEvent->pm10_std = (float)data.pm10_standard;
    WS_DEBUG_PRINT("PM10STD: ");
    WS_DEBUG_PRINTLN(pm10StdEvent->pm10_std);
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    PM25_AQI_Data data;
    if (!_pm25->read(&data)) {
      WS_DEBUG_PRINTLN("Failed to read PM25STD data");
      return false; // couldn't read data
    }
    pm25StdEvent->pm25_std = (float)data.pm25_standard;
    WS_DEBUG_PRINT("PM25STD: ");
    WS_DEBUG_PRINTLN(pm25StdEvent->pm25_std);
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    PM25_AQI_Data data;
    if (!_pm25->read(&data)) {
      WS_DEBUG_PRINTLN("Failed to read PM100STD data");
      return false; // couldn't read data
    }

    pm100StdEvent->pm100_std = (float)data.pm100_standard;
    WS_DEBUG_PRINT("PM100STD: ");
    WS_DEBUG_PRINTLN(pm100StdEvent->pm100_std);
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
  }

protected:
  Adafruit_PM25AQI *_pm25; ///< PM25 driver object
};

#endif // drvPm25