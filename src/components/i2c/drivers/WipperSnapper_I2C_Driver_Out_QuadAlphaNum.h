/*!
 * @file WipperSnapper_I2C_Driver_Out_QuadAlphaNum.h
 *
 *  Device driver for Quad Alphanumeric Displays w/I2C Backpack
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell for Adafruit Industries 2025
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_I2C_DRIVER_OUT_QUADALPHANUM_H
#define WIPPERSNAPPER_I2C_DRIVER_OUT_QUADALPHANUM_H

#include "WipperSnapper_I2C_Driver_Out.h"
#include <Adafruit_LEDBackpack.h>
#include <Arduino.h>

#define LED_BACKPACK_ALIGNMENT_UNSPECIFIED 0 ///< Unspecified alignment
#define LED_BACKPACK_ALIGNMENT_LEFT 1        ///< Left alignment
#define LED_BACKPACK_ALIGNMENT_RIGHT 2       ///< Right alignment
#define LED_BACKPACK_ALIGNMENT_DEFAULT                                         \
  LED_BACKPACK_ALIGNMENT_LEFT ///< Default alignment
#define LED_MAX_CHARS                                                          \
  4 ///< Maximum number of characters to display on the alphanumeric display


/*!
    @brief  Class that provides a driver interface for Quad Alphanumeric
   Displays w/I2C Backpack
*/
class WipperSnapper_I2C_Driver_Out_QuadAlphaNum : public WipperSnapper_I2C_Driver_Out {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an MS8607 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_Out_QuadAlphaNum(TwoWire *i2c, uint16_t sensorAddress) : WipperSnapper_I2C_Driver_Out(i2c, sensorAddress) {
    // No-op constructor
  }

  /*!
      @brief    Destructor for an MS8607 sensor.
  */
  ~WipperSnapper_I2C_Driver_Out_QuadAlphaNum() { 
    if (_alpha4 != nullptr) {
      delete _alpha4;
      _alpha4 = nullptr;
    }
   }

  /*!
      @brief    Initializes the drvOutQuadAlphaNum component and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _alpha4 = new Adafruit_AlphaNum4();
    bool did_begin = _alpha4->begin(_address, _i2c);
    _alpha4->setBrightness(_brightness);
    return did_begin;
  }

  /*!
    @brief    Configures a LED backpack.
    @param    brightness
              The brightness of the LED backpack.
    @param    alignment
              The alignment of the LED backpack.
*/
  void ConfigureI2CBackpack(int32_t brightness, uint32_t alignment) {
    if (alignment == LED_BACKPACK_ALIGNMENT_RIGHT) {
      _alignment = LED_BACKPACK_ALIGNMENT_RIGHT;
    } else {
      _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
    }
    _brightness = brightness;
  }



protected:
  Adafruit_AlphaNum4 *_alpha4 = nullptr;         ///< ptr to a 4-digit alphanumeric display object
  int32_t _brightness; ///< Brightness of the LED backpack, from 0 (off) to 15
                       ///< (full brightness)
  uint32_t _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT; ///< Determines L/R alignment of the message displayed
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_QUADALPHANUM_H