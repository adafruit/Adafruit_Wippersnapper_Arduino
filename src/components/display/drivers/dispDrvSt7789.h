/*!
 * @file src/components/display/drivers/dispDrvSt7789.h
 *
 * Driver for ST7789-based TFT displays (V2).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISP_DRV_ST7789_H
#define WS_DISP_DRV_ST7789_H

#include "dispDrvBase.h"
#include <Adafruit_ST7789.h>

/*!
    @brief  Driver for ST7789-based TFT displays.
*/
class dispDrvSt7789 : public dispDrvBase {
public:
  dispDrvSt7789(int16_t cs, int16_t dc, int16_t mosi, int16_t sck,
                int16_t rst = -1, int16_t miso = -1)
      : dispDrvBase(cs, dc, mosi, sck, rst, miso), _display(nullptr) {}

  ~dispDrvSt7789() {
    if (_display) {
      _display->fillScreen(ST77XX_BLACK);
      delete _display;
      _display = nullptr;
    }
    if (_pin_bl >= 0)
      digitalWrite(_pin_bl, LOW);
  }

  bool begin() override {
    _display = new Adafruit_ST7789(_pin_cs, _pin_dc, _pin_mosi, _pin_sck, _pin_rst);
    if (!_display)
      return false;

    // init() expects native (portrait) dimensions — smaller value first
    uint16_t native_w = min(_width, _height);
    uint16_t native_h = max(_width, _height);
    _display->init(native_w, native_h);
    _display->setRotation(_rotation);
    _display->fillScreen(ST77XX_BLACK);
    _display->setTextColor(ST77XX_WHITE);
    _display->setTextWrap(true);
    _display->setTextSize(_text_sz);

    // Turn on backlight
    if (_pin_bl >= 0) {
      pinMode(_pin_bl, OUTPUT);
      digitalWrite(_pin_bl, HIGH);
      WS_DEBUG_PRINTLN("[display] Backlight ON");
    }

    // Test: draw directly to confirm display works
    _display->setCursor(0, 0);
    _display->print("Display init OK");
    WS_DEBUG_PRINTLN("[display] ST7789 initialized");
    return true;
  }

  void writeMessage(const char *message, bool clear_first = true,
                    int32_t cursor_x = 0, int32_t cursor_y = 0) override {
    if (!_display)
      return;

    if (clear_first) {
      _display->fillScreen(ST77XX_BLACK);
    }

    _display->setCursor(cursor_x, cursor_y);
    _display->setTextSize(_text_sz);

    size_t msg_len = strlen(message);
    int16_t line_height = 8 * _text_sz;

    for (size_t i = 0; i < msg_len; i++) {
      // Handle escaped newlines: \n and \r\n
      if (message[i] == '\\' && i + 1 < msg_len) {
        if (message[i + 1] == 'r' && i + 3 < msg_len &&
            message[i + 2] == '\\' && message[i + 3] == 'n') {
          cursor_y += line_height;
          _display->setCursor(0, cursor_y);
          i += 3;
          continue;
        }
        if (message[i + 1] == 'n') {
          cursor_y += line_height;
          _display->setCursor(0, cursor_y);
          i++;
          continue;
        }
      }
      // Degree symbol (UTF-8: 0xC2 0xB0)
      if ((uint8_t)message[i] == 0xC2 && i + 1 < msg_len &&
          (uint8_t)message[i + 1] == 0xB0) {
        _display->write(char(247));
        i++;
        continue;
      }
      _display->print(message[i]);
    }
  }

private:
  Adafruit_ST7789 *_display;
};

#endif // WS_DISP_DRV_ST7789_H
