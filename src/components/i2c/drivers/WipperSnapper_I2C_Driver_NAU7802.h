/*!
 * @file WipperSnapper_I2C_Driver_NAU7802.h
 *
 * Device driver for the NAU7802 24bit ADC / load cell breakout
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry for Adafruit Industries 2024
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_NAU7802_H
#define WipperSnapper_I2C_Driver_NAU7802_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_NAU7802.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the NAU7802.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_NAU7802 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an NAU7802.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_NAU7802(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _nau7802 = new Adafruit_NAU7802();
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an NAU7802.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_NAU7802() { _nau7802 = nullptr; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the NAU7802 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    if (!_nau7802->begin(_i2c))
      return false;

    if (!_nau7802->setLDO(NAU7802_3V0)) {
      WS_DEBUG_PRINTLN("Failed to set LDO to 3V0");
    }

    if (!_nau7802->setGain(NAU7802_GAIN_128)) {
      WS_DEBUG_PRINTLN("Failed to set gain to 128");
    }

    if (!_nau7802->setRate(NAU7802_RATE_10SPS)) {
      WS_DEBUG_PRINTLN("Failed to set sample rate to 10SPS");
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's raw "force" value.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the reading was obtained successfully, False otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    unsigned long start = millis();

    // Wait for the sensor to be ready
    while (!_nau7802->available()) {
      if (millis() - start > 1000) {
        WS_DEBUG_PRINTLN("NAU7802 not available");
        return false;
      }
    }
    rawEvent->data[0] = (float)_nau7802->read();
    return true;
  }

protected:
  Adafruit_NAU7802 *_nau7802 = nullptr; ///< NAU7802 object
};

#endif // WipperSnapper_I2C_Driver_NAU7802_H