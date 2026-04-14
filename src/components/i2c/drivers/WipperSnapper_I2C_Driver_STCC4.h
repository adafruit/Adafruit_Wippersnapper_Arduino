/*!
 * @file WipperSnapper_I2C_Driver_STCC4.h
 *
 * Device driver for the STCC4 CO2, Temperature, and Humidity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
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
      @brief    Initializes the STCC4 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _stcc4 = new Adafruit_STCC4();
    if (!_stcc4->begin((uint8_t)_sensorAddress, _i2c))
      return false;
    // Enable continuous measurement mode for periodic reading
    if (!_stcc4->enableContinuousMeasurement(true))
      return false;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads all sensor data from the STCC4, caching the results.
                Only performs a new I2C read if the last read was more than
                one second ago.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    unsigned long now = millis();
    if (_lastRead != 0 && now - _lastRead < ONE_SECOND_IN_MILLIS)
      return true;

    uint16_t co2;
    float temperature, humidity;
    uint16_t status;

    if (!_stcc4->readMeasurement(&co2, &temperature, &humidity, &status))
      return false;

    _cachedCO2 = co2;
    _cachedTemperature = temperature;
    _cachedHumidity = humidity;
    _lastRead = now;
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

    tempEvent->temperature = _cachedTemperature;
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

    humidEvent->relative_humidity = _cachedHumidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current CO2 reading.
      @param    co2Event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the CO2 value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    if (!ReadSensorData())
      return false;

    co2Event->CO2 = (float)_cachedCO2;
    return true;
  }

protected:
  Adafruit_STCC4 *_stcc4 = nullptr; ///< STCC4 driver object
  ulong _lastRead = 0uL;            ///< Last time the sensor was read
  float _cachedTemperature = 0.0f;  ///< Cached temperature reading
  float _cachedHumidity = 0.0f;     ///< Cached humidity reading
  uint16_t _cachedCO2 = 0;          ///< Cached CO2 reading in ppm
};

#endif // WipperSnapper_I2C_Driver_STCC4_H
