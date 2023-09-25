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
  ~WipperSnapper_I2C_Driver_ENS160() { delete _ens160; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the ENS160 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _ens160 = new ScioSense_ENS160((TwoWire *)_i2c, (uint8_t)_sensorAddress);

    // attempt to initialize ENS160
    if (!_ens160->begin())
      return false;

    // Set the mode to standard
    return _ens160->setMode(ENS160_OPMODE_STD);
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a reading in blocking mode.
      @returns  True if the reading succeeded, False otherwise.
  */
  /*******************************************************************************/
  bool ensPerformReading() {
    return _ens160->available() && _ens160->measure(true);
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
    eco2Event->eCO2 = (float)_ens160->geteCO2();
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
    tvocEvent->tvoc = (float)_ens160->getTVOC();
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
    rawEvent->data[0] = (float)_ens160->getAQI();
    return true;
  }

protected:
  ScioSense_ENS160 *_ens160; ///< ENS160 object
};

#endif // WipperSnapper_I2C_Driver_ENS160