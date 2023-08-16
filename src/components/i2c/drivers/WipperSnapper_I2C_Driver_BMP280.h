/*!
 * @file WipperSnapper_I2C_Driver_BMP280.h
 *
 * Device driver for an AHT Humidity and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_BMP280_H
#define WipperSnapper_I2C_Driver_BMP280_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BMP280 temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BMP280 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BMP280 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BMP280(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BMP280 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_BMP280() { delete _bmp; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BMP280 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _bmp = new Adafruit_BMP280(_i2c);
    // attempt to initialize BMP280
    if (!_bmp->begin(_sensorAddress))
      return false;

    // set up sampling as recommended in Adafruit library
    _bmp->setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // configure BMP280 device
    _bmp_temp = _bmp->getTemperatureSensor();
    _bmp_pressure = _bmp->getPressureSensor();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BMP280's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (_bmp_temp == NULL)
      return false;
    _bmp_temp->getEvent(tempEvent);
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
    if (_bmp_pressure == NULL)
      return false;
    _bmp_pressure->getEvent(pressureEvent);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a the BMP280's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    altitudeEvent->altitude = _bmp->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BMP280 *_bmp; ///< BMP280  object
  Adafruit_Sensor *_bmp_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_bmp_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
  Adafruit_Sensor *_bmp_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
};

#endif // WipperSnapper_I2C_Driver_BMP280