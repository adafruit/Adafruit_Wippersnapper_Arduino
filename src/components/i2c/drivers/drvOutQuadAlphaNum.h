/*!
 * @file drvQuadAlphaNum.h
 *
 * Device driver for Quad Alphanumeric Displays w/I2C Backpack
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

#ifndef DRV_OUT_QUAD_ALPHANUM_H
#define DRV_OUT_QUAD_ALPHANUM_H

#include "drvOutputBase.h"
#include <Adafruit_LEDBackpack.h>

/*!
    @brief  Class that provides a driver interface for Quad Alphanumeric
   Displays w/I2C Backpack
*/
class drvOutQuadAlphaNum : public drvOutputBase {
public:
  /*!
      @brief    Constructor for a quad alphanumeric display..
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvOutQuadAlphaNum(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                     const char *driver_name)
      : drvOutputBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for a quad alphanumeric display.
  */
  ~drvOutQuadAlphaNum() {
    if (_alpha4) {
      delete _alpha4;
      _alpha4 = nullptr;
    }
  }

  /*!
      @brief    Initializes the drvOutQuadAlphaNum sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _alpha4 = new Adafruit_AlphaNum4();
    return _alpha4->begin(_address, _i2c);
  }

  /*!
      @brief    Writes the first four characters of a message to the quad
     alphanumeric display.
      @param    message
                  The message to be displayed.
  */
  void WriteMessage(const char *message) {
    if (_alpha4 == nullptr) {
      return;
    }
    for (size_t i = 0; i < 4; i++) {
      _alpha4->writeDigitAscii(i, message[i]);
    }
    _alpha4->writeDisplay();
  }

  /*!
      @brief    Writes a floating point value to the quad alphanumeric display.
      @param    value
                  The value to be displayed. Only the first four digits are
      displayed.
  */
  void WriteValue(float value) {
    if (_alpha4 == nullptr) {
      return;
    }
    // TODO!
  }

  /*!
      @brief    Writes an integer value to the quad alphanumeric display.
      @param    value
                  The value to be displayed. Only the first four digits are
      displayed.
  */
 void WriteValue(int32_t value) {
  if (_alpha4 == nullptr) {
    return;
  }
  // TODO!
}

protected:
  Adafruit_AlphaNum4 *_alpha4 =
      nullptr; ///< ptr to a 4-digit alphanumeric display object
};

#endif // DRV_OUT_QUAD_ALPHANUM_H