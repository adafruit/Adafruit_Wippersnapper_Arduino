/*!
 * @file src/components/display/drivers/dispDrvThinkInkGrayscale4MFGN.h
 *
 * Driver for ThinkInk 4.2" Grayscale 4-level MFGN display (SSD1683)
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
#ifndef WS_DRV_THINKINK_GRAYSCALE4_MFGN_H
#define WS_DRV_THINKINK_GRAYSCALE4_MFGN_H

#include "dispDrvBase.h"

/*!
    @brief  Driver for a ThinkInk 4.2" Grayscale 4-level MFGN display.
*/
class dispDrvThinkInkGrayscale4MFGN : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the ThinkInk Grayscale 4-level MFGN EPD display
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
  dispDrvThinkInkGrayscale4MFGN(int16_t dc, int16_t rst, int16_t cs,
                               int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  ~dispDrvThinkInkGrayscale4MFGN() {
    if (_display) {
      _display->clearBuffer();
      _display->display();
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the ThinkInk Grayscale 4-level MFGN EPD
              display driver.
      @param  mode
              The ThinkInk mode to use for the display.
      @param  reset
              Whether to reset the display before initialization.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin(thinkinkmode_t mode, bool reset = true) override {
    (void)reset;

    _display = new ThinkInk_420_Grayscale4_MFGN(_pin_dc, _pin_rst, _pin_cs,
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

  /*!
      @brief  Draws a status bar at the top of the display.
      @param  io_username
              The Adafruit IO username to display on the status bar.
  */
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

  /*!
    @brief  Updates the status bar with current information.
    @param  rssi
            The current WiFi RSSI (signal strength) in dB.
    @param  bat
            The current battery level as a percentage (0-100).
    @param  mqtt_status
            The current MQTT connection status.
  */
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    (void)bat;

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
      if (rssi >= -50) {
        wifi_icon = epd_bmp_wifi_full;
      } else if (rssi < -50 && rssi >= -60) {
        wifi_icon = epd_bmp_wifi_fair;
      } else if (rssi < -60 && rssi >= -70) {
        wifi_icon = epd_bmp_wifi_weak;
      } else {
        wifi_icon = epd_bmp_wifi_no_signal;
      }

      _display->fillRect(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ,
                           EPD_BLACK);
      _statusbar_rssi = rssi;
    }

    _display->display();
  }

  /*!
      @brief  Writes a message to the display.
      @param  message
              The message to write to the display.
  */
  void writeMessage(const char *message) override {
    if (_display == nullptr)
      return;

    _display->fillRect(0, STATUS_BAR_HEIGHT, _display->width(),
                       _display->height() - STATUS_BAR_HEIGHT, EPD_WHITE);

    int16_t y_idx = STATUS_BAR_HEIGHT + 4;
    _display->setCursor(0, y_idx);

    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);

    _display->setTextSize(_text_sz);

    for (size_t i = 0; i < msg_size && c_idx < msg_size; i++) {
      if (y_idx + line_height > _height)
        break;

      if (message[i] == '\\' && i + 1 < msg_size &&
          (message[i + 1] == 'n' || message[i + 1] == 'r')) {
        if (message[i + 1] == 'r' && i + 3 < msg_size && message[i + 2] == '\\' &&
            message[i + 3] == 'n') {
          if (y_idx + line_height > _height)
            break;
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i += 3;
        } else if (message[i + 1] == 'n') {
          if (y_idx + line_height > _height)
            break;
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i++;
        }
      } else if (message[i] == 0xC2 && i + 1 < msg_size && message[i + 1] == 0xB0) {
        _display->write(char(247));
        i++;
      } else {
        _display->print(message[i]);
      }

      c_idx++;
    }

    _display->display();
  }

private:
  ThinkInk_420_Grayscale4_MFGN *_display;
};

#endif // WS_DRV_THINKINK_GRAYSCALE4_MFGN_H
