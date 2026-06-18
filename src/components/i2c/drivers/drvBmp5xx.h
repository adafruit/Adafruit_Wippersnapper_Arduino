/*!
 * @file drvBmp5xx.h
 *
 * Device driver for a BMP5XX precision pressure sensor breakout.
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

#ifndef DRV_BMP5XX_H
#define DRV_BMP5XX_H

#include "drvBase.h"
#include <Adafruit_BMP5xx.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BMP5XX temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvBmp5xx : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BMP5XX sensor.
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
  drvBmp5xx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _bmp5xx = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BMP5XX sensor.
  */
  /*******************************************************************************/
  ~drvBmp5xx() {
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
  bool begin() override {
    _bmp5xx = new Adafruit_BMP5xx();
    if (!_bmp5xx->begin(_address, _i2c)) {
      delete _bmp5xx;
      _bmp5xx = nullptr;
      return false;
    }

    // CUSTOM SETTINGS CANDIDATES (currently hard-coded defaults below; not yet
    // exposed via the v2 properties API):
    //  - Temperature/pressure oversampling (setTemperatureOversampling /
    //    setPressureOversampling) trade noise for conversion time.
    //  - IIR filter coefficient (setIIRFilterCoeff) smooths short-term noise.
    //  - Output data rate (setOutputDataRate) and power mode (setPowerMode).
    return _bmp5xx->setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X) &&
           _bmp5xx->setPressureOversampling(BMP5XX_OVERSAMPLING_16X) &&
           _bmp5xx->setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3) &&
           _bmp5xx->setOutputDataRate(BMP5XX_ODR_50_HZ) &&
           _bmp5xx->setPowerMode(BMP5XX_POWERMODE_NORMAL) &&
           _bmp5xx->enablePressure(true);
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
    if (!_bmp5xx->performReading()) {
      return false;
    }
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
    if (!_bmp5xx->performReading()) {
      return false;
    }
    pressureEvent->pressure = _bmp5xx->pressure;
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
    if (!_bmp5xx->performReading()) {
      return false;
    }
    altitudeEvent->altitude = _bmp5xx->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BMP5xx *_bmp5xx; ///< BMP5xx object
};

#endif // DRV_BMP5XX_H
