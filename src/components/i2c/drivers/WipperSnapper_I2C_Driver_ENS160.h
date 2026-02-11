/*!
 * @file WipperSnapper_I2C_Driver_ENS160.h
 *
 * Device driver for a ENS160 MOX Gas Sensor.
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

#ifndef WipperSnapper_I2C_Driver_ENS160_H
#define WipperSnapper_I2C_Driver_ENS160_H

#include "WipperSnapper_I2C_Driver.h"
#include <ScioSense_ENS160.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the ENS160 temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_ENS160 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an ENS160 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_ENS160(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an ENS160 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_ENS160() { delete _ens16x; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the ENS160 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ens16x = new ScioSense_ENS160((TwoWire *)_i2c, (uint8_t)_sensorAddress);

    // attempt to initialize ENS16x, verify chip id
    if (!_ens16x->begin() || !_ens16x->available())
      return false;

    /* In future set the mode to ulp for 161 (need to add to adafruit lib), see
     * https://github.com/sciosense/ens16x-arduino/blob/d09d25dd0912b729a21366e58b55393a49afc256/src/lib/ens16x/ScioSense_Ens161.h#L10-L22
     * _ens16x->revENS16x() == 0 ? ENS160_OPMODE_STD : ENS160_OPMODE_LP/ULP
     */
    return _ens16x->setMode(ENS160_OPMODE_STD);
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a reading in blocking mode.
      @returns  True if the reading succeeded, False otherwise.
  */
  /*******************************************************************************/
  bool ensPerformReading() {
    return _ens16x->available() && _ens16x->measure(true);
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the ENS160's eCO2 sensor into an event.
      @param    eco2Event
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventECO2(sensors_event_t *eco2Event) {
    if (!ensPerformReading())
      return false;
    eco2Event->eCO2 = (float)_ens16x->geteCO2();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the ENS160's TVOC sensor into an event.
      @param    tvocEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventTVOC(sensors_event_t *tvocEvent) {
    if (!ensPerformReading())
      return false;
    tvocEvent->tvoc = (float)_ens16x->getTVOC();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the ENS160's AQI value into an event.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!ensPerformReading())
      return false;
    rawEvent->data[0] = (float)_ens16x->getAQI();
    return true;
  }

protected:
  ScioSense_ENS160 *_ens16x; ///< ENS160/1 object
};

#endif // WipperSnapper_I2C_Driver_ENS160