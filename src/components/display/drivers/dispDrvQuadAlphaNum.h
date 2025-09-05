/*!
 * @file src/components/display/drivers/dispDrvQuadAlphaNum.h
 *
 * Driver for Quad Alphanumeric 7-Segment LED Backpack displays.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISP_DRV_QUADALPHANUM
#define WS_DISP_DRV_QUADALPHANUM

#include "dispDrvBase.h"
#include <Adafruit_LEDBackpack.h>

#define LED_BACKPACK_ALIGNMENT_UNSPECIFIED 0 ///< Unspecified alignment
#define LED_BACKPACK_ALIGNMENT_LEFT 1        ///< Left alignment
#define LED_BACKPACK_ALIGNMENT_RIGHT 2       ///< Right alignment
#define LED_BACKPACK_ALIGNMENT_DEFAULT                                         \
  LED_BACKPACK_ALIGNMENT_LEFT ///< Default alignment
#define LED_MAX_CHARS                                                          \
  4 ///< Maximum number of characters to display on the alphanumeric display
#define ALPHANUM_DEGREE_SYMBOL                                                 \
  0b0000000011100011 ///< Degree symbol for alphanumeric display

/*!
    @brief  Driver for Quad Alphanumeric LED matrixes.
*/
class dispDrvQuadAlphaNum : public dispDrvBase {
public:
  /*!
      @brief  Constructor for Quad Alphanumeric LED matrixes.
        @param  i2c
                The I2C hardware interface, default is Wire.
        @param  sensorAddress
                The I2C sensor's unique address.
  */
  dispDrvQuadAlphaNum(TwoWire *i2c, uint16_t sensorAddress)
      : dispDrvBase(i2c, sensorAddress), _alpha4(nullptr) {
    _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
  }

  /*!
      @brief  Destructor for the 7-segment LED backpack driver.
  */
  ~dispDrvQuadAlphaNum() {
    if (_alpha4) {
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
    bool did_begin = _alpha4->begin(_sensorAddress, _i2c);
    _alpha4->setBrightness(_brightness);
    return did_begin;
  }

  /*!
      @brief    Sets the brightness of the LED backpack.
      @param    b
                  The brightness value, from 0 (off) to 15 (full brightness).
  */
  void setBrightness(int32_t brightness) override {
    if (_alpha4 == nullptr) {
      return;
    }
    _alpha4->setBrightness(brightness);
  }

  /*!
      @brief    Sets the alignment of the displayed text.
      @param    alignment
                  The alignment value, either LED_BACKPACK_ALIGNMENT_LEFT or
                  LED_BACKPACK_ALIGNMENT_RIGHT.
  */
  void setAlignment(uint32_t alignment) override {
    if (alignment == LED_BACKPACK_ALIGNMENT_RIGHT) {
      _alignment = LED_BACKPACK_ALIGNMENT_RIGHT;
    } else {
      _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
    }
  }

  /*!
      @brief    Writes the first four characters of a message to the quad
     alphanumeric display.
      @param    message
                  The message to be displayed.
  */
  void writeMessage(const char *message) override {
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
      } else if (message[i] == 0xC2 && message[i + 1] == 0xB0 &&
                 i + 1 < strlen(message)) {
        // Write the degree symbol
        _alpha4->writeDigitRaw(cur_idx, ALPHANUM_DEGREE_SYMBOL);
        i++;
        cur_idx++;
        continue;
      }
      // Write the character to the display buffer
      _alpha4->writeDigitAscii(cur_idx, ch, display_dot);
      cur_idx++;
    }
    // Issue the buffered data in RAM to the display
    _alpha4->writeDisplay();
  }

private:
  Adafruit_AlphaNum4 *_alpha4; ///< Pointer to an Adafruit AlphaNum4 object
};

#endif // WS_DISP_DRV_QUADALPHANUM