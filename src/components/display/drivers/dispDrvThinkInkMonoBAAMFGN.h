/*!
 * @file src/components/display/drivers/dispDrvThinkInkMonoBAAMFGN.h
 *
 * Driver for ThinkInk 3.7" Monochrome BAAMFGN display (ADA6395)
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
#ifndef WS_DRV_THINKINK_MONO_BAAMFGN_H
#define WS_DRV_THINKINK_MONO_BAAMFGN_H

#include "dispDrvBase.h"

/*!
    @brief  Driver for a ThinkInk 3.7" Monochrome BAAMFGN display (ADA6395).
*/
class dispDrvThinkInkMonoBAAMFGN : public dispDrvBase {
public:
  dispDrvThinkInkMonoBAAMFGN(int16_t dc, int16_t rst, int16_t cs,
                             int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  ~dispDrvThinkInkMonoBAAMFGN() {
    if (_display) {
      _display->clearBuffer();
      _display->display();
      delete _display;
      _display = nullptr;
    }
  }

  bool begin(thinkinkmode_t mode, bool reset = true) override {
    _display = new ThinkInk_370_Mono_BAAMFGN(_pin_dc, _pin_rst, _pin_cs,
                                             _pin_sram_cs, _pin_busy);
    if (!_display)
      return false;
    _display->begin(mode);
    _display->setTextSize(_text_sz);
    _display->setTextColor(EPD_BLACK);
    _display->setTextWrap(false);
    _height = _display->height();
    _width = _display->width();
    _display->clearBuffer();
    _display->display();
    return true;
  }

  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;
    _display->clearBuffer();
    _display->fillRect(0, 0, _display->width(), STATUS_BAR_HEIGHT, EPD_BLACK);
    _display->fillRect(STATUS_BAR_BORDER, STATUS_BAR_BORDER,
                       _display->width() - (2 * STATUS_BAR_BORDER),
                       STATUS_BAR_HEIGHT - (2 * STATUS_BAR_BORDER), EPD_WHITE);
    _display->setTextSize(1);
    _display->setTextColor(EPD_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);
    _statusbar_icons_y =
        STATUS_BAR_BORDER +
        ((STATUS_BAR_HEIGHT - 2 * STATUS_BAR_BORDER - STATUS_BAR_ICON_SZ) / 2);
    _statusbar_icon_battery_x =
        _display->width() - STATUS_BAR_ICON_SZ - STATUS_BAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x - STATUS_BAR_ICON_SZ -
                             STATUS_BAR_ICON_SPACING;
    _statusbar_icon_cloud_x =
        _statusbar_icon_wifi_x - STATUS_BAR_ICON_SZ - STATUS_BAR_ICON_SPACING;
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, STATUS_BAR_ICON_SZ,
                         STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, STATUS_BAR_ICON_SZ,
                         STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, STATUS_BAR_ICON_SZ,
                         STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->display();
  }

  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    if (!_display)
      return;
    bool update_rssi = abs(rssi - _statusbar_rssi) >= 5;
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;
    if (!update_rssi && !update_mqtt)
      return;
    if (update_mqtt) {
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_WHITE);
      _display->drawBitmap(
          _statusbar_icon_cloud_x, _statusbar_icons_y,
          mqtt_status ? epd_bmp_cloud_online : epd_bmp_cloud_offline,
          STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
      _statusbar_mqtt_connected = mqtt_status;
    }
    if (update_rssi) {
      const unsigned char *wifi_icon = epd_bmp_wifi_no_signal;
      if (rssi >= -50)
        wifi_icon = epd_bmp_wifi_full;
      else if (rssi >= -60)
        wifi_icon = epd_bmp_wifi_fair;
      else if (rssi >= -70)
        wifi_icon = epd_bmp_wifi_weak;
      _display->fillRect(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ,
                           EPD_BLACK);
      _statusbar_rssi = rssi;
    }
    _display->display();
  }

  void writeMessage(const char *message, bool clear_first = true,
                    int32_t cursor_x = 0, int32_t cursor_y = 0) override {
    if (!_display)
      return;
    if (clear_first) {
      _display->fillRect(0, STATUS_BAR_HEIGHT, _display->width(),
                         _display->height() - STATUS_BAR_HEIGHT, EPD_WHITE);
    }
    int16_t y_idx =
        (cursor_y > 0) ? cursor_y : (STATUS_BAR_HEIGHT + 4);
    _display->setCursor(cursor_x, y_idx);
    int16_t line_height = 8 * _text_sz;
    size_t msg_size = strlen(message);
    _display->setTextSize(_text_sz);
    for (size_t i = 0; i < msg_size; i++) {
      if (y_idx + line_height > _height)
        break;
      if (message[i] == '\\' && i + 1 < msg_size) {
        if (message[i + 1] == 'r' && i + 3 < msg_size &&
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
      if ((uint8_t)message[i] == 0xC2 && i + 1 < msg_size &&
          (uint8_t)message[i + 1] == 0xB0) {
        _display->write(char(247));
        i++;
        continue;
      }
      _display->print(message[i]);
    }
    _display->display();
  }

private:
  ThinkInk_370_Mono_BAAMFGN *_display;
};

#endif // WS_DRV_THINKINK_MONO_BAAMFGN_H
