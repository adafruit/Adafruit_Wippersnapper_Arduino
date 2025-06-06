/*!
 * @file WipperSnapper_I2C_Driver_Out_7Seg.h
 *
 * Device driver designed specifically to work with the Adafruit LED 7-Segment
 * I2C backpacks:
 * ----> http://www.adafruit.com/products/881
 * ----> http://www.adafruit.com/products/880
 * ----> http://www.adafruit.com/products/879
 * ----> http://www.adafruit.com/products/878
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

#ifndef WIPPERSNAPPER_I2C_DRIVER_OUT_7SEG_H
#define WIPPERSNAPPER_I2C_DRIVER_OUT_7SEG_H

#include "WipperSnapper_I2C_Driver_Out.h"
#include <Adafruit_LEDBackpack.h>
#include <Arduino.h>

#define LED_BACKPACK_ALIGNMENT_UNSPECIFIED 0 ///< Unspecified alignment
#define LED_BACKPACK_ALIGNMENT_LEFT 1        ///< Left alignment
#define LED_BACKPACK_ALIGNMENT_RIGHT 2       ///< Right alignment
#define LED_BACKPACK_ALIGNMENT_DEFAULT                                         \
  LED_BACKPACK_ALIGNMENT_LEFT ///< Default alignment
#define LED_MAX_CHARS 5

/*!
    @brief  Class that provides a driver interface for 7-Segment
   Displays w/I2C Backpack
*/
class WipperSnapper_I2C_Driver_Out_7Seg : public WipperSnapper_I2C_Driver_Out {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a 7-Segment display driver.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_Out_7Seg(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver_Out(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*!
      @brief    Destructor for a 7-Segment display driver.
  */
  ~WipperSnapper_I2C_Driver_Out_7Seg() {
    if (_matrix != nullptr) {
      delete _matrix;
      _matrix = nullptr;
    }
  }

  /*!
      @brief    Initializes the 7-segment LED matrix and begins I2C
     communication.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() {
    _matrix = new Adafruit_7segment();
    bool did_begin = _matrix->begin(_sensorAddress, _i2c);
    return did_begin;
  }

  /*!
    @brief    Configures the LED matrix's I2C backpack.
    @param    brightness
              The brightness of the i2c backpack (0-15).
    @param    alignment
              The alignment of the i2c backpack's LED matrix (left/right).
*/
  void ConfigureI2CBackpack(int32_t brightness, uint32_t alignment) {
    if (alignment == LED_BACKPACK_ALIGNMENT_RIGHT) {
      _alignment = LED_BACKPACK_ALIGNMENT_RIGHT;
    } else {
      _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
    }

    // Note: Adafruit_LEDBackpack normalizes only > 15, but not < 0,
    // so, here we're normalizing < 0 to 8 (median brightness)
    if (brightness < 0) {
      brightness = 8; // Set to median brightness if out of range
    }
  }

  /*!
      @brief    Sets the brightness of the LED backpack.
      @param    b
                  The brightness value, from 0 (off) to 15 (full brightness).
  */
  void SetLedBackpackBrightness(uint8_t b) {
    if (_matrix == nullptr) {
      return;
    }
    _matrix->setBrightness(b);
  }

  /*!
      @brief    Writes the first four characters of a message to the Adafruit
     7-segment LED matrix.
      @param    message
                  The message to be displayed.
  */
  void WriteMessage(const char *message) {
    if (_matrix == nullptr || message == nullptr) {
      return;
    }

    // Clear before writing
    _matrix->clear();

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
      switch (seg_chars) {
      case 4:
        pos_start = 0;
        break;
      case 3:
        pos_start = 1;
        break;
      case 2:
        pos_start = 3; // if 2 characters, start at position 3 is required
                       // because ':' is position 2 and we need to skip it
        break;
      case 1:
        pos_start = 4;
        break;
      default:
        pos_start = 0; // if no characters or overflow, start at position 0
        break;
      }
    }

    // Write to the display's buffer
    int cur_idx = pos_start;
    for (size_t i = 0; i < len_display; i++) {

      // skip position 2
      if (cur_idx == 2) {
        cur_idx++;
      }
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
      _matrix->writeDigitAscii(cur_idx, ch, display_dot);
      cur_idx++;
    }
    // Issue the buffered data in RAM to the display
    _matrix->writeDisplay();
  }

protected:
  Adafruit_7segment *_matrix =
      nullptr;         ///< ptr to a 7-segment LED matrix object
  int32_t _brightness; ///< Brightness of the LED backpack, from 0 (off) to 15
                       ///< (full brightness)
  uint32_t _alignment =
      LED_BACKPACK_ALIGNMENT_DEFAULT; ///< Determines L/R alignment of the
                                      ///< message displayed
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_7SEG_H