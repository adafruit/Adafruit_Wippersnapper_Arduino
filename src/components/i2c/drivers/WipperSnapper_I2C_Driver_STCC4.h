/*!
 * @file WipperSnapper_I2C_Driver_STCC4.h
 *
 * Device driver for the STCC4 CO2, Temperature, and Humidity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Melissa LeBlanc-Williams 2026 for Adafruit Industries.
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
    if (!_stcc4->begin((uint8_t)_sensorAddress, _i2c)) {
      return false;
    }

    // Use continuous measurement mode with 1s interval (library default use)
    if (!_stcc4->enableContinuousMeasurement(true)) {
      return false;
    }

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
    if (!_readSensor()) {
      return false;
    }

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
    if (!_readSensor()) {
      return false;
    }

    *humidEvent = _humidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current CO2 reading.
      @param    co2Event
                Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    if (!_readSensor()) {
      return false;
    }

    *co2Event = _CO2;
    return true;
  }

protected:
  Adafruit_STCC4 *_stcc4 = nullptr;   ///< STCC4 driver object
  sensors_event_t _temperature = {0}; ///< Temperature
  sensors_event_t _humidity = {0};    ///< Relative Humidity
  sensors_event_t _CO2 = {0};         ///< CO2
  unsigned long _lastRead = 0;        ///< Last time the sensor was read

  /*******************************************************************************/
  /*!
      @brief    Reads sensor data, with a 1-second cache to avoid redundant
                I2C reads when multiple getEvent*() calls occur per cycle.
      @returns  True if cached data is available or a fresh read succeeded.
  */
  /*******************************************************************************/
  bool _readSensor() {
    if (_lastRead != 0 && millis() - _lastRead < 1000) {
      return true;
    }

    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    uint16_t status = 0;

    if (!_stcc4 ||
        !_stcc4->readMeasurement(&co2, &temperature, &humidity, &status)) {
      return false;
    }

    _CO2.CO2 = co2;
    _temperature.temperature = temperature;
    _humidity.relative_humidity = humidity;
    _lastRead = millis();
    return true;
  }
};

#endif // WipperSnapper_I2C_Driver_STCC4_H
