/*!
 * @file WipperSnapper_I2C_Driver_RotaryEncoder.h
 *
 * Device driver for the STEMMA QT Rotary Encoder.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_RotaryEncoder_H
#define WipperSnapper_I2C_Driver_RotaryEncoder_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_Seesaw.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a STEMMA QT Rotary Encoder.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_RotaryEncoder : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a STEMMA QT Rotary Encoder.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_RotaryEncoder(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c; //TODO: Is this necessary if it doesn't need to get passed to seesaw::begin?
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an STEMMA QT Rotary Encoder.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_RotaryEncoder() {
    // Called when a RotaryEncoder component is deleted.
    delete _RotaryEncoder;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the STEMMA QT Rotary Encoder and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _RotaryEncoder = new Adafruit_seesaw();
    return _RotaryEncoder->begin((uint8_t)_sensorAddress); //TODO: Do i need to send constructor params for unit8_t "flow" or bool "reset"?
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the current postion of a rotary encoder.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEncoderPosition(sensors_event_t *rawEvent) {
    rawEvent->data = _RotaryEncoder->getEncoderPosition(); //it doesn't look like Adafruit_Sensor was built for this sort of thing
    return true;
  }



protected:
  Adafruit_seesaw *_RotaryEncoder; ///< Pointer to RotaryEncoder object
};

#endif // WipperSnapper_I2C_Driver_RotaryEncoder