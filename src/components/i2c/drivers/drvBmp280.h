/*!
 * @file drvBmp280.h
 *
 * Device driver for a BMP280 Pressure and Temperature sensor.
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

#ifndef DRV_BMP280_H
#define DRV_BMP280_H

#include "drvBase.h"
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/*!
    @brief  Class that provides a sensor driver for the BMP280 temperature
            and pressure sensor.
*/
class drvBmp280 : public drvBase {

public:
  /*!
      @brief    Constructor for an BMP280 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvBmp280(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an BMP280 sensor.
  */
  ~drvBmp280() { delete _bmp; }

  /*!
      @brief    Initializes the BMP280 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _bmp = new Adafruit_BMP280(_i2c);
    // attempt to initialize BMP280
    if (!_bmp->begin(_address))
      return false;

    // set up sampling as recommended in Adafruit library
    _bmp->setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // attempt to get sensors
    _bmp_temp = _bmp->getTemperatureSensor();
    if (_bmp_temp == NULL)
      return false;
    _bmp_pressure = _bmp->getPressureSensor();
    if (_bmp_pressure == NULL)
      return false;

    return true;
  }

  /*!
      @brief    Gets the BMP280's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _bmp_temp->getEvent(tempEvent);
  }

  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventPressure(sensors_event_t *pressureEvent) {
    return _bmp_pressure->getEvent(pressureEvent);
  }

  /*!
      @brief    Reads a the BMP280's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
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

#endif // drvBmp280