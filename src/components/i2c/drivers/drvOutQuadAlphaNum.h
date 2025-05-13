/*!
 * @file drvOutQuadAlphaNum.h
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
    WS_DEBUG_PRINTLN("[i2c] drvOutQuadAlphaNum::ConfigureI2CBackpack() called");
    if (alignment == LED_BACKPACK_ALIGNMENT_RIGHT) {
      _alignment = LED_BACKPACK_ALIGNMENT_RIGHT;
    } else {
      _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
    }
    _brightness = brightness;
    WS_DEBUG_PRINTLN("[i2c] drvOutQuadAlphaNum::configured");
  }

  /*!
      @brief    Sets the brightness of the LED backpack.
      @param    b
                  The brightness value, from 0 (off) to 15 (full brightness).
  */
  void SetLedBackpackBrightness(uint8_t b) {
    if (_alpha4 == nullptr) {
      return;
    }
    _alpha4->setBrightness(b);
  }

  /*!
      @brief    Writes the first four characters of a message to the quad
     alphanumeric display.
      @param    message
                  The message to be displayed.
  */
  void WriteMessage(const char *message) {
    if (_alpha4 == nullptr || message == nullptr) {
      return;
    }
    // Clear before writing
    _alpha4->clear();

    // Calculate the number of characters to display
    size_t len_display = min(strlen(message), (size_t)LED_MAX_CHARS);

    // Set the starting position based on alignment
    int pos_start;
    if (_alignment == LED_BACKPACK_ALIGNMENT_LEFT) {
      pos_start = 0; // start at the leftmost position of the display
    } else {
      // Exclude decimal points from the character count because those get
      // displayed on a "special" segment of the LED display
      int seg_chars = 0;
      for (size_t i = 0; i < len_display; i++) {
        if (message[i] != '.') {
          seg_chars++;
        }
      }
      // start at the rightmost position of the display
      pos_start = LED_MAX_CHARS - seg_chars;
    }

    // Write to the display's buffer
    int cur_idx = pos_start;
    for (size_t i = 0; i < len_display; i++) {
      // Save the character because if there's a decimal, we need to skip it in
      // the buffer
      char ch = message[i];

      // Look-ahead for a decimal point to attach to the current character
      bool display_dot = false;
      if (i + 1 < len_display && message[i + 1] == '.') {
        display_dot = true;
        i++;
        len_display++;
      }

      // Write the character to the display buffer
      _alpha4->writeDigitAscii(cur_idx, ch, display_dot);
      cur_idx++;
    }
    // Issue the buffered data in RAM to the display
    _alpha4->writeDisplay();
  }

  /*!
      @brief    Writes a floating point value to the quad alphanumeric display.
      @param    value
                  The value to be displayed. Only the first four digits are
      displayed.
  */
  void WriteValue(float value) {
    char message[8 + 1];
    snprintf(message, sizeof(message), "%.5f", value);
    WriteMessage(message);
  }

  /*!
      @brief    Writes an integer value to the quad alphanumeric display.
      @param    value
                  The value to be displayed. Only the first four digits are
      displayed.
  */
  void WriteValue(int32_t value) {
    char message[LED_MAX_CHARS + 1];
    snprintf(message, sizeof(message), "%ld", value);
    WriteMessage(message);
  }

protected:
  Adafruit_AlphaNum4 *_alpha4 =
      nullptr;         ///< ptr to a 4-digit alphanumeric display object
  int32_t _brightness; ///< Brightness of the LED backpack, from 0 (off) to 15
                       ///< (full brightness)
  uint32_t _alignment =
      LED_BACKPACK_ALIGNMENT_DEFAULT; ///< Determines L/R alignment of the
                                      ///< message displayed
};

#endif // DRV_OUT_QUAD_ALPHANUM_H