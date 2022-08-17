/*!
 * @file WipperSnapper_I2C_Driver_PM25.h
 *
 * Device driver for the Adafruit PM2.5 Air Quality Sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_PM25_H
#define WipperSnapper_I2C_Driver_PM25_h

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_PM25AQI.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the PM25 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_PM25 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a PM25 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_PM25(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the PM25 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _pm25 = new Adafruit_PM25AQI();

    // Wait one second for sensor to boot up!
    delay(1000);

    if (!_pm25->begin_I2C(_i2c))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the PM25 sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    PM25_AQI_Data data;
    if (!_pm25->read(&data))
      return false; // couldn't read data

    pm10StdEvent->data[0] = (float)data.pm10_standard;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the PM25 sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    PM25_AQI_Data data;
    if (!_pm25->read(&data))
      return false; // couldn't read data

    pm25StdEvent->data[0] = (float)data.pm25_standard;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the PM25 sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    PM25_AQI_Data data;
    if (!_pm25->read(&data))
      return false; // couldn't read data

    pm100StdEvent->data[0] = (float)data.pm100_standard;
    return true;
  }

protected:
  Adafruit_PM25AQI *_pm25; ///< PM25 driver object
};

#endif // WipperSnapper_I2C_Driver_PM25