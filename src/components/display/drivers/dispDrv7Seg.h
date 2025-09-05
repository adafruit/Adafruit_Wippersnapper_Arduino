/*!
 * @file src/components/display/drivers/dispDrv7Seg.h
 *
 * Driver for 7-Segment LED Backpack displays.
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
#ifndef WS_DISP_DRV_7Seg
#define WS_DISP_DRV_7Seg

#include "dispDrvBase.h"
#include <Adafruit_LEDBackpack.h>

#define LED_BACKPACK_ALIGNMENT_UNSPECIFIED 0 ///< Unspecified alignment
#define LED_BACKPACK_ALIGNMENT_LEFT 1        ///< Left alignment
#define LED_BACKPACK_ALIGNMENT_RIGHT 2       ///< Right alignment
#define LED_BACKPACK_ALIGNMENT_DEFAULT                                         \
  LED_BACKPACK_ALIGNMENT_LEFT ///< Default alignment
#define LED_MAX_CHARS                                                          \
  5 ///< Maximum characters for 7-segment display, including ':'
#define LED_BACKPACK_DEGREE_SYMBOL                                             \
  0b01100011 ///< Degree symbol for 7-segment display

/*!
    @brief  Driver for 7-segment LED backpack displays.
*/
class dispDrv7Seg : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the 7-segment LED matrix.
  */
  dispDrv7Seg(TwoWire *i2c, uint16_t sensorAddress)
      : dispDrvBase(i2c, sensorAddress), _matrix(nullptr) {
    _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
  }

  /*!
      @brief  Destructor for the 7-segment LED backpack driver.
  */
  ~dispDrv7Seg() {
    if (_matrix) {
      delete _matrix;
      _matrix = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the 7-segment LED backpack driver.
      @return True if the matrix was initialized successfully, false otherwise.
  */
  bool begin() override {
    _matrix = new Adafruit_7segment();
    return _matrix->begin(_sensorAddress, _i2c);
  }

  /*!
      @brief  Sets the text alignment for the matrix.
      @param  alignment
              The desired alignment to set (LEFT or RIGHT).
  */
  void setAlignment(uint32_t alignment) override {
    if (alignment == LED_BACKPACK_ALIGNMENT_RIGHT) {
      _alignment = LED_BACKPACK_ALIGNMENT_RIGHT;
    } else {
      _alignment = LED_BACKPACK_ALIGNMENT_DEFAULT;
    }
  }

  /*!
      @brief    Writes the first four characters of a message to the Adafruit
     7-segment LED matrix.
      @param    message
                  The message to be displayed.
  */
  void writeMessage(const char *message) override {
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
      } else if (message[i] == 0xC2 && message[i + 1] == 0xB0 &&
                 i + 1 < strlen(message)) {
        // Write degree symbol
        _matrix->writeDigitRaw(cur_idx, LED_BACKPACK_DEGREE_SYMBOL);
        i++;
        cur_idx++;
        continue; // skip to next character
      }
      // Write the character to the display buffer
      _matrix->writeDigitAscii(cur_idx, ch, display_dot);
      cur_idx++;
    }
    // Issue the buffered data in RAM to the display
    _matrix->writeDisplay();
  }

private:
  Adafruit_7segment *_matrix;
};

#endif // WS_DISP_DRV_7Seg