/*!
 * @file WipperSnapper_I2C_Driver_BMP5XX.h
 *
 * Device driver for a BMP5XX precision pressure sensor breakout.
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

#ifndef WipperSnapper_I2C_Driver_BMP5XX_H
#define WipperSnapper_I2C_Driver_BMP5XX_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BMP5XX.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BMP5XX temperature
            and pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BMP5XX : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BMP5XX sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BMP5XX(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _bmp5xx = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BMP5XX sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_BMP5XX() { 
    if (_bmp5xx) {
      delete _bmp5xx; 
      _bmp5xx = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BMP5XX sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _bmp5xx = new Adafruit_BMP5xx();
    if (!_bmp5xx->begin(_sensorAddress, _i2c))
      return false;

    // Set up oversampling and filter initialization
    _bmp5xx->setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    _bmp5xx->setPressureOversampling(BMP5XX_OVERSAMPLING_4X);
    _bmp5xx->setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    _bmp5xx->setOutputDataRate(BMP5XX_ODR_50_HZ);

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BMP5XX's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_bmp5xx->performReading())
      return false;
    tempEvent->temperature = _bmp5xx->temperature;
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
    if (!_bmp5xx->performReading())
      return false;
    pressureEvent->pressure = _bmp5xx->pressure / 10.0F;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a the BMP5XX's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    if (!_bmp5xx->performReading())
      return false;
    altitudeEvent->altitude = _bmp5xx->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BMP5xx *_bmp5xx; ///< BMP5xx object
};

#endif // WipperSnapper_I2C_Driver_BMP5XX