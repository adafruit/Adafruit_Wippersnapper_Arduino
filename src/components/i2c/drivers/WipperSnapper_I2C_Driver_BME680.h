/*!
 * @file WipperSnapper_I2C_Driver_BME680.h
 *
 * Device driver for an AHT Humidity and Temperature sensor.
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

#ifndef WipperSnapper_I2C_Driver_BME680_H
#define WipperSnapper_I2C_Driver_BME680_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BME680.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

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
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BME680(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BME680 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_BME680() { delete _bme; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BME680 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _bme = new Adafruit_BME680();
    bool isInit = _bme->begin(_sensorAddress, _i2c);
    // Set up oversampling and filter initialization
    _bme->setTemperatureOversampling(BME680_OS_8X);
    _bme->setHumidityOversampling(BME680_OS_2X);
    _bme->setPressureOversampling(BME680_OS_4X);
    _bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme->setGasHeater(320, 150); // 320*C for 150 ms
    return isInit;
  }

  /*******************************************************************************/
  /*******************************************************************************/
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    // Sample the bme680's sensors
    if (! _bme->performReading())
      return false;

    // Pack the sensors_event_t's temperature field
    // with the bme680's temperature value
    tempEvent->temperature = _bme->temperature;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BME680's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
  // Sample the bme680's sensors
  if (! _bme->performReading())
   return false;

  // Pack the sensors_event_t's humidity field
  // with the bme680's humidity value
  humidEvent->relative_humidity = _bme->humidity;

  return true;
}

/*******************************************************************************/
 /*!
     @brief    Reads a gas sensor

     @param    vocEvent
               Pointer to an Adafruit_Sensor event.
     @returns  True if the sensor event was obtained successfully, False
               otherwise.
 */
 /*******************************************************************************/

  bool getEventTvoc(sensors_event_t *tvocEvent) {
    // Sample the bme680's sensors
    // This is not working
    if (! _bme->performReading())
     return false;

    // Pack the sensors_event_t's gas field
    // with the bme680's gas value
    tvocEvent->data[0] = _bme->gas_resistance / 1000.0;
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
   // Sample the bme680's sensors
  if (! _bme->performReading())
    return false;

  // Pack the sensors_event_t's pressure field
  // with the bme680's pressure value
   pressureEvent->pressure = _bme->pressure / 100;
   return true;
}

  /*******************************************************************************/
  /*!
      @brief    Reads a the BME680's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    // TODO: Note, this is a hack into Adafruit_Sensor, we should really add an
    // altitude sensor type
    altitudeEvent->data[0] = _bme->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BME680 *_bme; ///< BME680  object
};

#endif // WipperSnapper_I2C_Driver_BME680
