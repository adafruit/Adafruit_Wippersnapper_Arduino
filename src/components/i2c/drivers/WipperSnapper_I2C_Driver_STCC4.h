/*!
 * @file WipperSnapper_I2C_Driver_STCC4.h
 *
 * Device driver for the STCC4 CO2, Temperature, and Humidity sensor.
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

#ifndef WipperSnapper_I2C_Driver_STCC4_H
#define WipperSnapper_I2C_Driver_STCC4_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_STCC4.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the STCC4 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_STCC4 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a STCC4 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_STCC4(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a STCC4 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_STCC4() { delete _stcc4; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the STCC4 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _stcc4 = new Adafruit_STCC4();
    if (!_stcc4->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    // Start continuous measurement (1s sampling interval)
    return _stcc4->enableContinuousMeasurement(true);
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  /*******************************************************************************/
  bool HasBeenReadInLastSecond() {
    return _lastRead != 0 && millis() - _lastRead < 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the STCC4 sensor and caches the results.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    // Don't read sensor more than once per second
    if (HasBeenReadInLastSecond())
      return true;

    uint16_t co2 = 0;
    float temperature = 0.0;
    float humidity = 0.0;
    uint16_t status = 0;
    if (!_stcc4->readMeasurement(&co2, &temperature, &humidity, &status))
      return false;

    _CO2.CO2 = (float)co2;
    _temperature.temperature = temperature;
    _humidity.relative_humidity = humidity;
    _lastRead = millis();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadSensorData())
      return false;
    *tempEvent = _temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (!ReadSensorData())
      return false;
    *humidEvent = _humidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current CO2 reading.
      @param    co2Event
                Pointer to an Adafruit_Sensor event for CO2.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    if (!ReadSensorData())
      return false;
    *co2Event = _CO2;
    return true;
  }

protected:
  Adafruit_STCC4 *_stcc4 = nullptr;   ///< STCC4 driver object
  ulong _lastRead = 0uL;              ///< Last time the sensor was read
  sensors_event_t _temperature = {0}; ///< Temperature
  sensors_event_t _humidity = {0};    ///< Relative Humidity
  sensors_event_t _CO2 = {0};         ///< CO2
};

#endif // WipperSnapper_I2C_Driver_STCC4_H
