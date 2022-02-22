/*!
 * @file WipperSnapper_I2C_Driver_BME280.h
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

#ifndef WipperSnapper_I2C_Driver_BME280_H
#define WipperSnapper_I2C_Driver_BME280_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BME280 temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BME280 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BME280 sensor.
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the BME280 sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BME280(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    isInitialized = _bme.begin(sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BME280 sensor.
  */
  /*******************************************************************************/
  virtual ~WipperSnapper_I2C_Driver_BME280() {
    _bme_temp = NULL;
    _bme_humidity = NULL;
    _bme_pressure = NULL;
    _tempSensorPeriod = 0.0L;
    _humidSensorPeriod = 0.0L;
    _pressureSensorPeriod = 0.0L;
    _altitudeSensorPeriod = 0.0L;
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the BME280's temperature sensor.
  */
  /*******************************************************************************/
  void enableSensorAmbientTemperature() {
    _bme_temp = _bme.getTemperatureSensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the BME280's humidity sensor.
  */
  /*******************************************************************************/
  void enableSensorRelativeHumidity() {
    _bme_humidity = _bme.getHumiditySensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the BME280's humidity sensor.
  */
  /*******************************************************************************/
  void enableSensorPressure() { _bme_pressure = _bme.getPressureSensor(); }

  /*******************************************************************************/
  /*!
      @brief    Disables the BME280's temperature sensor.
  */
  /*******************************************************************************/
  void disableSensorAmbientTemperature() {
    _bme_temp = NULL;
    _tempSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the BME280's humidity sensor.
  */
  /*******************************************************************************/
  void disableSensorRelativeHumidity() {
    _bme_humidity = NULL;
    _humidSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the BME280's pressure sensor.
  */
  /*******************************************************************************/
  void disableSensorPressure() {
    _bme_pressure = NULL;
    _pressureSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a pressure sensor.
      @param    period
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorPressure(float period) {
    // enable a previously disabled sensor
    if (period > 0 && _bme_pressure == NULL)
      enableSensorPressure();
    setSensorPressurePeriod(period);
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of an ambient temperature
                  sensor, provided sensor_period.
      @param    period
                Sensor's period.
  */
  /*******************************************************************************/
  void updateSensorAmbientTemperature(float period) {
    // disable the sensor
    if (period == 0)
      disableSensorAmbientTemperature();
    // enable a previously disabled sensor
    if (period > 0 && _bme_temp == NULL)
      enableSensorAmbientTemperature();

    setSensorAmbientTemperaturePeriod(period);
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a relative humidity sensor.
      @param    period
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  void updateSensorRelativeHumidity(float period) {
    // disable the sensor
    if (period == 0)
      disableSensorRelativeHumidity();
    // enable a previously disabled sensor
    if (period > 0 && _bme_humidity == NULL)
      enableSensorRelativeHumidity();

    setSensorRelativeHumidityPeriod(period);
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
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    if (_bme_temp == NULL)
      return false;
    _bme_temp->getEvent(tempEvent);
    return true;
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
    if (_bme_humidity == NULL)
      return false;
    _bme_humidity->getEvent(humidEvent);
    return true;
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
    if (_bme_pressure == NULL)
      return false;
    _bme_pressure->getEvent(pressureEvent);
    return true;
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
    // TODO: Note, this is a hack into Adafruit_Sensor, we should really add an
    // altitude sensor type
    altitudeEvent->data[0] = _bme.readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BME280 _bme; ///< BME280  object
  Adafruit_Sensor *_bme_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_bme_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
  Adafruit_Sensor *_bme_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
};

#endif // WipperSnapper_I2C_Driver_BME280