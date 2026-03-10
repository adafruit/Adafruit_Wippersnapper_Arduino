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

#define ST7789_STATUSBAR_HEIGHT 20  ///< Status bar height in pixels
#define ST7789_STATUSBAR_ICON_SZ 16 ///< Status bar icon size in pixels
#define ST7789_STATUSBAR_ICON_SPACING 4 ///< Spacing between icons
#define ST7789_STATUSBAR_ICON_MARGIN 5  ///< Margin from edge of display

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
    _display =
        new Adafruit_ST7789(_pin_cs, _pin_dc, _pin_mosi, _pin_sck, _pin_rst);
    if (!_display)
      return false;

    // init() expects native (portrait) dimensions — smaller value first
    uint16_t native_w = min(_width, _height);
    uint16_t native_h = max(_width, _height);
    _display->init(native_w, native_h);
    _display->setRotation(_rotation);
    _display->fillScreen(ST77XX_BLACK);
    _display->setTextColor(ST77XX_WHITE);
    _display->setTextWrap(false);
    _display->setTextSize(_text_sz);

    // Turn on backlight
    if (_pin_bl >= 0) {
      pinMode(_pin_bl, OUTPUT);
      digitalWrite(_pin_bl, HIGH);
      WS_DEBUG_PRINTLN("[display] Backlight ON");
    }

    WS_DEBUG_PRINTLN("[display] ST7789 initialized");
    return true;
  }

  void showSplash() override {
    if (!_display)
      return;

    if (_width == 240 && _height == 240) {
      _display->drawBitmap(0, 0, tft_bmp_logo_240240, 240, 240, ST77XX_WHITE);
    } else if (_width == 135 && _height == 240) {
      _display->drawBitmap(0, 0, tft_bmp_logo_240135, 240, 135, ST77XX_WHITE);
    } else {
      return; // Unsupported resolution, skip splash
    }

    delay(500);
  }

  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;

    // Clear entire display to remove splash screen
    _display->fillScreen(ST77XX_BLACK);

    // Draw white status bar at top
    _display->fillRect(0, 0, _display->width(), ST7789_STATUSBAR_HEIGHT,
                       ST77XX_WHITE);

    // Draw username on left side
    _display->setTextSize(1);
    _display->setTextColor(ST77XX_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);

    // Calculate icon positions (right-aligned), centered vertically
    _statusbar_icons_y =
        (ST7789_STATUSBAR_HEIGHT - ST7789_STATUSBAR_ICON_SZ) / 2;
    _statusbar_icon_battery_x = _display->width() - ST7789_STATUSBAR_ICON_SZ -
                                ST7789_STATUSBAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x -
                             ST7789_STATUSBAR_ICON_SZ -
                             ST7789_STATUSBAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x -
                              ST7789_STATUSBAR_ICON_SZ -
                              ST7789_STATUSBAR_ICON_SPACING;

    // Draw icons
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, ST7789_STATUSBAR_ICON_SZ,
                         ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, ST7789_STATUSBAR_ICON_SZ,
                         ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, ST7789_STATUSBAR_ICON_SZ,
                         ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);

    // Reset text color and size for main text area
    _display->setTextColor(ST77XX_WHITE);
    _display->setTextSize(_text_sz);
  }

  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    if (!_display)
      return;

    bool update_rssi = abs(rssi - _statusbar_rssi) >= 3;
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;

    if (!update_rssi && !update_mqtt)
      return;

    if (update_mqtt) {
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                         ST77XX_WHITE);
      if (mqtt_status) {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_online, ST7789_STATUSBAR_ICON_SZ,
                             ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);
      } else {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_offline, ST7789_STATUSBAR_ICON_SZ,
                             ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);
      }
      _statusbar_mqtt_connected = mqtt_status;
    }

    if (update_rssi) {
      const unsigned char *wifi_icon = epd_bmp_wifi_no_signal;
      if (rssi >= -50) {
        wifi_icon = epd_bmp_wifi_full;
      } else if (rssi >= -60) {
        wifi_icon = epd_bmp_wifi_fair;
      } else if (rssi >= -70) {
        wifi_icon = epd_bmp_wifi_weak;
      }
      _display->fillRect(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                         ST77XX_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, ST7789_STATUSBAR_ICON_SZ,
                           ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);
      _statusbar_rssi = rssi;
    }
  }

  void writeMessage(const char *message, bool clear_first = true,
                    int32_t cursor_x = 0, int32_t cursor_y = 0) override {
    if (!_display)
      return;

    // Default: clear only below status bar, start text below status bar
    if (clear_first) {
      _display->fillRect(0, ST7789_STATUSBAR_HEIGHT, _display->width(),
                         _display->height() - ST7789_STATUSBAR_HEIGHT,
                         ST77XX_BLACK);
    }

    int16_t y_idx =
        (cursor_y > 0) ? cursor_y : ST7789_STATUSBAR_HEIGHT + 5;
    int16_t line_height = 8 * _text_sz;

    _display->setTextSize(_text_sz);
    _display->setCursor(cursor_x, y_idx);

    size_t msg_len = strlen(message);
    for (size_t i = 0; i < msg_len; i++) {
      // Stop if we'd overflow the display
      if (y_idx + line_height > _height)
        break;

      char parsed_char = 0;
      bool is_newline = false;
      if (parseWriteToken(message, msg_len, i, parsed_char, is_newline,
                          char(247))) {
        if (is_newline) {
          y_idx += line_height;
          if (y_idx + line_height > _height)
            break;
          _display->setCursor(0, y_idx);
        } else {
          _display->write(parsed_char);
        }
        continue;
      }
      _display->print(message[i]);
    }
  }

private:
  Adafruit_ST7789 *_display;
};

#endif // WS_DISP_DRV_ST7789_H
