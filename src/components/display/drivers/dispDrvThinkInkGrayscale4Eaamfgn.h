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
  drvDispThinkInkGrayscale4Eaamfgn(int16_t dc, int16_t rst, int16_t cs,
                                   int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  ~drvDispThinkInkGrayscale4Eaamfgn() {
    if (_display) {
      _display->clearBuffer();
      _display->display();
      delete _display;
      _display = nullptr;
    }
  }

  bool begin(thinkinkmode_t mode, bool reset = true) override {
    _display = new ThinkInk_290_Grayscale4_EAAMFGN(_pin_dc, _pin_rst, _pin_cs,
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
    return true;
  }

  void showSplash() override {
    if (!_display)
      return;
    _display->drawBitmap(0, 0, epd_bitmap_ws_logo_296128, 296, 128, EPD_BLACK);
    _display->display();
  }

  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;
    _display->clearBuffer();
    _display->fillRect(0, 0, _display->width(), EPD_STATUS_BAR_HEIGHT, EPD_BLACK);
    _display->fillRect(EPD_STATUS_BAR_BORDER, EPD_STATUS_BAR_BORDER,
                       _display->width() - (2 * EPD_STATUS_BAR_BORDER),
                       EPD_STATUS_BAR_HEIGHT - (2 * EPD_STATUS_BAR_BORDER), EPD_WHITE);
    _display->setTextSize(1);
    _display->setTextColor(EPD_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);
    _statusbar_icons_y =
        EPD_STATUS_BAR_BORDER +
        ((EPD_STATUS_BAR_HEIGHT - 2 * EPD_STATUS_BAR_BORDER - EPD_STATUS_BAR_ICON_SZ) / 2);
    _statusbar_icon_battery_x =
        _display->width() - EPD_STATUS_BAR_ICON_SZ - EPD_STATUS_BAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x - EPD_STATUS_BAR_ICON_SZ -
                             EPD_STATUS_BAR_ICON_SPACING;
    _statusbar_icon_cloud_x =
        _statusbar_icon_wifi_x - EPD_STATUS_BAR_ICON_SZ - EPD_STATUS_BAR_ICON_SPACING;
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
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
                         EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ, EPD_WHITE);
      _display->drawBitmap(
          _statusbar_icon_cloud_x, _statusbar_icons_y,
          mqtt_status ? epd_bmp_cloud_online : epd_bmp_cloud_offline,
          EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
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
                         EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ, EPD_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ,
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
      _display->fillRect(0, EPD_STATUS_BAR_HEIGHT, _display->width(),
                         _display->height() - EPD_STATUS_BAR_HEIGHT, EPD_WHITE);
    }
    int16_t y_idx =
        (cursor_y > 0) ? cursor_y : (EPD_STATUS_BAR_HEIGHT + 4);
    _display->setCursor(cursor_x, y_idx);
    int16_t line_height = 8 * _text_sz;
    size_t msg_size = strlen(message);
    _display->setTextSize(_text_sz);
    for (size_t i = 0; i < msg_size; i++) {
      if (y_idx + line_height > _height)
        break;
      char parsed_char = 0;
      bool is_newline = false;
      if (parseWriteToken(message, msg_size, i, parsed_char, is_newline,
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
    _display->display();
  }

private:
  ThinkInk_290_Grayscale4_EAAMFGN *_display;
};

#endif // WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H
