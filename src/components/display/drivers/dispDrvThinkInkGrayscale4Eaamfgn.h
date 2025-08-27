/*!
 * @file src/components/display/drivers/dispDrvThinkInkGrayscale4Eaamfgn.h
 *
 * Driver for ThinkInk 2.9" Grayscale 4-level EAAMFGN display (present on the
 * 2025 version of the Adafruit MagTag)
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
#ifndef WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H
#define WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H

#include "dispDrvBase.h"

/*!
    @brief  Driver for a ThinkInk 2.9" Grayscale 4-level EAAMFGN display.
*/
class drvDispThinkInkGrayscale4Eaamfgn : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the ThinkInk Grayscale 4-level EAAMFGN display
               driver.
      @param  dc
              Data/Command pin for the display.
      @param  rst
              Reset pin for the display.
      @param  cs
              Chip Select pin for the display.
      @param  sram_cs
              Optional SRAM Chip Select pin for E-Ink displays that support it.
      @param  busy
              Optional Busy pin for the display.
  */
  drvDispThinkInkGrayscale4Eaamfgn(int16_t dc, int16_t rst, int16_t cs,
                                   int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  ~drvDispThinkInkGrayscale4Eaamfgn() {
    if (_display) {
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the ThinkInk Grayscale 4-level EAAMFGN
              display driver.
      @param  mode
              The ThinkInk mode to use for the display.
      @param  reset
              Whether to reset the display before initialization.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin(thinkinkmode_t mode, bool reset = true) override {
    _display = new ThinkInk_290_Grayscale4_EAAMFGN(_pin_dc, _pin_rst, _pin_cs,
                                                   _pin_sram_cs, _pin_busy);
    if (!_display)
      return false; // Allocation failed

    // Initialize the display
    _display->begin(mode);
    // Configure display settings
    _text_sz = 3;
    _display->setTextSize(_text_sz);
    _display->setTextColor(EPD_BLACK);
    _display->setTextWrap(false);
    _height = _display->height();
    _width = _display->width();
    // Clear the display buffer
    _display->clearBuffer();
    _display->display();

    return true;
  }

  /*!
      @brief  Writes a message to the display.
      @param  message
              The message to write to the display.
      @note   This method overrides the base class method to provide specific
              functionality for the Think Ink Grayscale 4 EAAMGFGN driver.
  */
  virtual void writeMessage(const char *message) override {
    if (_display == nullptr)
      return;

    // Start with a fresh display buffer
    _display->clearBuffer();
    int16_t y_idx = 0;
    _display->setCursor(0, y_idx);

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);
    for (size_t i = 0; i < msg_size && c_idx < msg_size; i++) {
      if (y_idx + line_height > _height)
        break;
      if (message[i] == '\\' && i + 1 < msg_size &&
          (message[i + 1] == 'n' || message[i + 1] == 'r')) {
        // Handle \r\n sequence as a single newline
        if (message[i + 1] == 'r' && i + 3 < msg_size &&
            message[i + 2] == '\\' && message[i + 3] == 'n') {
          // Skip to the next line
          if (y_idx + line_height > _height)
            break;
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i += 3;
        } else if (message[i + 1] == 'n') {
          // Skip to the next line
          if (y_idx + line_height > _height)
            break;
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i++;
        }
      } else if (message[i] == 0xC2 && message[i + 1] == 0xB0) {
        // Degree symbol
        _display->write(char(248));
        i++;
      } else {
        _display->print(message[i]);
      }
    }
    _display->display();
  }

private:
  ThinkInk_290_Grayscale4_EAAMFGN *_display;
};

#endif // WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H