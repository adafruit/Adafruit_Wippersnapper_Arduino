/*!
 * @file WipperSnapper_I2C_Driver_BME680.h
 *
 * Device driver for an AHT Humidity and Temperature sensor.
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

#ifndef WipperSnapper_I2C_Driver_BME680_H
#define WipperSnapper_I2C_Driver_BME680_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BME680.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BME680 temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BME680 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BME680 sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the BME680 sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BME680(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    setDriverType(BME680); // sets the type of I2C_Driver
    _isInitialized = _bme.begin(sensorAddress, _i2c);
    if (!_isInitialized)
      return;
    // Set up oversampling and filter initialization
    _bme->setTemperatureOversampling(BME680_OS_8X);
    _bme->setHumidityOversampling(BME680_OS_2X);
    _bme->setPressureOversampling(BME680_OS_4X);
    _bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme->setGasHeater(320, 150); // 320*C for 150 ms
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BME680 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_BME680() {
    _tempSensorPeriod = 0.0L;
    _humidSensorPeriod = 0.0L;
    _pressureSensorPeriod = 0.0L;
    _bme_gas = 0;
    setDriverType(UNSPECIFIED);
  }

  // NOTE: updateSensorX() calls are handled in the base class, I2C_Driver
  // NOTE: enableSensorX() calls are handled in the base class, I2C_Driver
  // NOTE: disableSensorX() calls are handled in the base class, I2C_Driver

  /*******************************************************************************/
  /*!
      @brief    Reads the BME680's temperature sensor.
      @param    tempEvent
                A temperature value.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemperature(float tempEvent) {
    if (!_bme->performReading())
      return false;
    tempEvent = _bme.temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the BME680's humidity sensor.
      @param    humidEvent
                A humidity value
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(float humidEvent) {
    if (!_bme->performReading())
      return false;
    humidEvent = _bme.humidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the BME680's pressure sensor.
      @param    pressureEvent
                A pressure value
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(float pressureEvent) {
    if (!bme->performReading())
      return false;
    pressureEvent = _bme.pressure / 100.0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the BME680's gas sensor and converts
                the reading into the expected SI unit.
      @param    gasReading
                Gas resistor (ohms) reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventGas(uint32_t gas_resistance;) {
    if (!bme->performReading())
      return false;
    gas_resistance = _bme.gas_resistance / 1000.0;
    return true;
  }

protected:
  Adafruit_BME680 _bme;   ///< BME680  object
  float _bme_temperature; ///< Temperature (C) assigned after reading
  float _bme_humidity;    ///< Humidity (RH%) assigned after reading
  float _bme_pressure;    ///< Pressure (Pascals) assigned after reading
  uint32_t _bme_gas;      ///< Gas resistance (ohms) assigned after reading

  Adafruit_Sensor *_bme_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_bme_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
  Adafruit_Sensor *_bme_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
};

#endif // WipperSnapper_I2C_Driver_BME680