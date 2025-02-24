/*!
 * @file drvBmp3xx.h
 *
 * Device driver for a BMP3XX precision pressure sensor breakout.
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

#ifndef DRV_BMP3XX_H
#define DRV_BMP3XX_H

#include "drvBase.h"
#include <Adafruit_BMP3XX.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BMP3XX temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvBmp3xx : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BMP3XX sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvBmp3xx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BMP3XX sensor.
  */
  /*******************************************************************************/
  ~drvBmp3xx() { delete _bmp3xx; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BMP3XX sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _bmp3xx = new Adafruit_BMP3XX();
    // attempt to initialize BMP3XX
    if (!_bmp3xx->begin_I2C(_address, _i2c))
      return false;

    // Set up oversampling and filter initialization
    if (!_bmp3xx->setTemperatureOversampling(BMP3_OVERSAMPLING_8X))
      return false;
    if (!_bmp3xx->setPressureOversampling(BMP3_OVERSAMPLING_4X))
      return false;
    if (!_bmp3xx->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3))
      return false;
    if (!_bmp3xx->setOutputDataRate(BMP3_ODR_50_HZ))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BMP3XX's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_bmp3xx->performReading())
      return false;
    tempEvent->temperature = _bmp3xx->temperature;
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
    if (!_bmp3xx->performReading())
      return false;
    pressureEvent->pressure = _bmp3xx->pressure / 100.0F;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a the BMP3XX's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    if (!_bmp3xx->performReading())
      return false;
    altitudeEvent->altitude = _bmp3xx->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BMP3XX *_bmp3xx; ///< BMP3XX  object
};
#endif // drvBmp3xx