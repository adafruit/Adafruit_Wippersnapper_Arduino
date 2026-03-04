/*!
 * @file src/components/display/drivers/dispDrvRgb666.h
 *
 * Driver for RGB666 dotclock displays via Arduino_GFX (V2).
 * Targets the Adafruit Qualia ESP32-S3 board with ST7701S-based panels.
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
#ifndef WS_DISP_DRV_RGB666_H
#define WS_DISP_DRV_RGB666_H

#ifdef ARDUINO_ADAFRUIT_QUALIA_S3_RGB666

#include "dispDrvBase.h"
#include <Arduino_GFX_Library.h>

#define RGB666_STATUSBAR_HEIGHT 20  ///< Status bar height in pixels
#define RGB666_STATUSBAR_ICON_SZ 16 ///< Status bar icon size in pixels
#define RGB666_STATUSBAR_ICON_SPACING 4 ///< Spacing between icons
#define RGB666_STATUSBAR_ICON_MARGIN 5  ///< Margin from edge of display

/*!
    @brief  Driver for RGB666 dotclock displays on Qualia ESP32-S3.
*/
class dispDrvRgb666 : public dispDrvBase {
public:
  /*!
      @brief  Constructor — panel selection by name string.
      @param  panel  Panel identifier (e.g. "TL021WVC02", "TL032FWV01").
  */
  dispDrvRgb666(const char *panel)
      : dispDrvBase(), _display(nullptr), _expander(nullptr),
        _rgbpanel(nullptr) {
    strncpy(_panel, panel, sizeof(_panel) - 1);
    _panel[sizeof(_panel) - 1] = '\0';
  }

  ~dispDrvRgb666() {
    if (_display) {
      _display->fillScreen(RGB565_BLACK);
      delete _display;
      _display = nullptr;
    }
    if (_expander) {
      _expander->digitalWrite(PCA_TFT_BACKLIGHT, LOW);
    }
    // rgbpanel and expander are owned by display, cleaned up by its destructor
  }

  bool begin() override {
    Wire.setClock(400000);

    _expander = new Arduino_XCA9554SWSPI(PCA_TFT_RESET, PCA_TFT_CS,
                                         PCA_TFT_SCK, PCA_TFT_MOSI, &Wire,
                                         0x3F);
    if (!_expander)
      return false;

    _rgbpanel = new Arduino_ESP32RGBPanel(
        TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK, TFT_R1, TFT_R2, TFT_R3,
        TFT_R4, TFT_R5, TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
        TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,
        1 /* hsync_polarity */, 46 /* hsync_front_porch */,
        2 /* hsync_pulse_width */, 44 /* hsync_back_porch */,
        1 /* vsync_polarity */, 50 /* vsync_front_porch */,
        16 /* vsync_pulse_width */, 16 /* vsync_back_porch */);
    if (!_rgbpanel)
      return false;

    // Select init operations based on panel string
    const uint8_t *init_ops = nullptr;
    size_t init_ops_len = 0;

    if (strcmp(_panel, "TL021WVC02") == 0) {
      init_ops = TL021WVC02_init_operations;
      init_ops_len = sizeof(TL021WVC02_init_operations);
    } else if (strcmp(_panel, "TL032FWV01") == 0) {
      init_ops = tl032fwv01_init_operations;
      init_ops_len = sizeof(tl032fwv01_init_operations);
    } else {
      WS_DEBUG_PRINT("[display] ERROR: Unknown RGB666 panel: ");
      WS_DEBUG_PRINTLN(_panel);
      return false;
    }

    _display = new Arduino_RGB_Display(_width, _height, _rgbpanel,
                                       _rotation, true /* auto_flush */,
                                       _expander, GFX_NOT_DEFINED /* RST */,
                                       init_ops, init_ops_len);
    if (!_display)
      return false;

    if (!_display->begin()) {
      WS_DEBUG_PRINTLN("[display] ERROR: Arduino_RGB_Display begin() failed!");
      return false;
    }

    _display->fillScreen(RGB565_BLACK);
    _display->setTextColor(RGB565_WHITE);
    _display->setTextWrap(false);
    _display->setTextSize(_text_sz);

    // Enable backlight via IO expander
    _expander->pinMode(PCA_TFT_BACKLIGHT, OUTPUT);
    _expander->digitalWrite(PCA_TFT_BACKLIGHT, HIGH);

    WS_DEBUG_PRINT("[display] RGB666 panel '");
    WS_DEBUG_PRINT(_panel);
    WS_DEBUG_PRINTLN("' initialized");
    return true;
  }

  void showSplash() override {
    // No splash bitmap for these panels yet
  }

  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;

    _display->fillScreen(BLACK);

    // Draw white status bar at top
    _display->fillRect(0, 0, _display->width(), RGB666_STATUSBAR_HEIGHT,
                       RGB565_WHITE);

    // Draw username on left side
    _display->setTextSize(1);
    _display->setTextColor(RGB565_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);

    // Calculate icon positions (right-aligned)
    _statusbar_icons_y =
        (RGB666_STATUSBAR_HEIGHT - RGB666_STATUSBAR_ICON_SZ) / 2;
    _statusbar_icon_battery_x = _display->width() - RGB666_STATUSBAR_ICON_SZ -
                                RGB666_STATUSBAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x -
                             RGB666_STATUSBAR_ICON_SZ -
                             RGB666_STATUSBAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x -
                              RGB666_STATUSBAR_ICON_SZ -
                              RGB666_STATUSBAR_ICON_SPACING;

    // Draw icons
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, RGB666_STATUSBAR_ICON_SZ,
                         RGB666_STATUSBAR_ICON_SZ, BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, RGB666_STATUSBAR_ICON_SZ,
                         RGB666_STATUSBAR_ICON_SZ, BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, RGB666_STATUSBAR_ICON_SZ,
                         RGB666_STATUSBAR_ICON_SZ, BLACK);

    // Reset text color and size for main text area
    _display->setTextColor(RGB565_WHITE);
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
                         RGB666_STATUSBAR_ICON_SZ, RGB666_STATUSBAR_ICON_SZ,
                         RGB565_WHITE);
      if (mqtt_status) {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_online, RGB666_STATUSBAR_ICON_SZ,
                             RGB666_STATUSBAR_ICON_SZ, RGB565_BLACK);
      } else {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_offline, RGB666_STATUSBAR_ICON_SZ,
                             RGB666_STATUSBAR_ICON_SZ, RGB565_BLACK);
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
                         RGB666_STATUSBAR_ICON_SZ, RGB666_STATUSBAR_ICON_SZ,
                         RGB565_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, RGB666_STATUSBAR_ICON_SZ,
                           RGB666_STATUSBAR_ICON_SZ, RGB565_BLACK);
      _statusbar_rssi = rssi;
    }
  }

  void writeMessage(const char *message, bool clear_first = true,
                    int32_t cursor_x = 0, int32_t cursor_y = 0) override {
    if (!_display)
      return;

    if (clear_first) {
      _display->fillRect(0, RGB666_STATUSBAR_HEIGHT, _display->width(),
                         _display->height() - RGB666_STATUSBAR_HEIGHT, RGB565_BLACK);
    }

    int16_t y_idx =
        (cursor_y > 0) ? cursor_y : RGB666_STATUSBAR_HEIGHT + 5;
    int16_t line_height = 8 * _text_sz;

    _display->setTextSize(_text_sz);
    _display->setCursor(cursor_x, y_idx);

    size_t msg_len = strlen(message);
    for (size_t i = 0; i < msg_len; i++) {
      if (y_idx + line_height > _height)
        break;

      // Handle escaped newlines: \r\n and \n
      if (message[i] == '\\' && i + 1 < msg_len) {
        if (message[i + 1] == 'r' && i + 3 < msg_len &&
            message[i + 2] == '\\' && message[i + 3] == 'n') {
          y_idx += line_height;
          if (y_idx + line_height > _height)
            break;
          _display->setCursor(0, y_idx);
          i += 3;
          continue;
        }
        if (message[i + 1] == 'n') {
          y_idx += line_height;
          if (y_idx + line_height > _height)
            break;
          _display->setCursor(0, y_idx);
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
  Arduino_RGB_Display *_display;
  Arduino_XCA9554SWSPI *_expander;
  Arduino_ESP32RGBPanel *_rgbpanel;
  char _panel[20];
};

#endif // ARDUINO_ADAFRUIT_QUALIA_S3_RGB666

#endif // WS_DISP_DRV_RGB666_H
