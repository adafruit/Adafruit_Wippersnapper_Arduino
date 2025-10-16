/*!
 * @file src/components/display/drivers/dispDrvThinkInkGrayscale4T5.h
 *
 * Driver for ThinkInk 2.9" Grayscale 4-level T5 display (present on the
 * pre-2025 version of the Adafruit MagTag)
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
#ifndef WS_DRV_THINKINK_GRAYSCALE4_T5_H
#define WS_DRV_THINKINK_GRAYSCALE4_T5_H

#include "dispDrvBase.h"

/*!
    @brief  Driver for a ThinkInk 2.9" Grayscale 4-level T5 display (pre-2025
   version of the Adafruit MagTag).
*/
class dispDrvThinkInkGrayscale4T5 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the ThinkInk Grayscale T5 EPD display driver.
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
  dispDrvThinkInkGrayscale4T5(int16_t dc, int16_t rst, int16_t cs,
                              int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  ~dispDrvThinkInkGrayscale4T5() {
    if (_display) {
      // Clear the display buffer before deleting
      _display->clearBuffer();
      _display->display();
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the ThinkInk Grayscale 4 T5 EPD
              display driver.
      @param  mode
              The ThinkInk mode to use for the display.
      @param  reset
              Whether to reset the display before initialization.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin(thinkinkmode_t mode, bool reset = true) override {
    _display = new ThinkInk_290_Grayscale4_T5(_pin_dc, _pin_rst, _pin_cs,
                                              _pin_sram_cs, _pin_busy);
    if (!_display)
      return false; // Allocation failed

    // Initialize the display
    _display->begin(mode);
    // Configure display settings
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
      @brief  Draws a status bar at the top of the display.
        @param  io_username
                The Adafruit IO username to display on the status bar.
  */
  virtual void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;

    // Clear the entire display buffer to remove splash screen
    _display->clearBuffer();

    // Draw status bar
    _display->fillRect(0, 0, _display->width(), STATUS_BAR_HEIGHT, EPD_BLACK);
    _display->fillRect(STATUS_BAR_BORDER, STATUS_BAR_BORDER,
                       _display->width() - (2 * STATUS_BAR_BORDER),
                       STATUS_BAR_HEIGHT - (2 * STATUS_BAR_BORDER), EPD_WHITE);

    // Draw username on left side of the status bar
    _display->setTextSize(1);
    _display->setTextColor(EPD_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);

    // Calculate status bar icon positions and center vertically
    _statusbar_icons_y =
        STATUS_BAR_BORDER +
        ((STATUS_BAR_HEIGHT - 2 * STATUS_BAR_BORDER - STATUS_BAR_ICON_SZ) / 2);
    _statusbar_icon_battery_x =
        _display->width() - STATUS_BAR_ICON_SZ - STATUS_BAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x - STATUS_BAR_ICON_SZ -
                             STATUS_BAR_ICON_SPACING;
    _statusbar_icon_cloud_x =
        _statusbar_icon_wifi_x - STATUS_BAR_ICON_SZ - STATUS_BAR_ICON_SPACING;
    // Draw icons on right side of the status bar
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
  @brief  Updates the status bar with current information (battery level,
  connectivity status, etc).
  @param  rssi
          The current WiFi RSSI (signal strength) in dB.
  @param  bat
          The current battery level as a percentage (0-100).
  @param  mqtt_status
          The current MQTT connection status.
*/
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    if (!_display)
      return;

    // Only update wifi icon if the RSSI has changed significantly (+/- 5dB)
    bool update_rssi = abs(rssi - _statusbar_rssi) >= 5;
    // Only update cloud icon if MQTT status has changed
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;

    // No need to update if nothing has changed
    if (!update_rssi && !update_mqtt)
      return;

    if (update_mqtt) {
      // updating the RSSI occurs too frequently to be practical
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_WHITE);
      if (mqtt_status) {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_online, STATUS_BAR_ICON_SZ,
                             STATUS_BAR_ICON_SZ, EPD_BLACK);
      } else {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_offline, STATUS_BAR_ICON_SZ,
                             STATUS_BAR_ICON_SZ, EPD_BLACK);
      }
      _statusbar_mqtt_connected = mqtt_status;
    }

    // Update WiFi icon only if RSSI has changed significantly (+/-3dB)
    if (update_rssi) {
      const unsigned char *wifi_icon = epd_bmp_wifi_no_signal;
      if (rssi >= -50) {
        wifi_icon = epd_bmp_wifi_full;
      } else if (rssi < -50 && rssi >= -60) {
        wifi_icon = epd_bmp_wifi_fair;
      } else if (rssi < -60 && rssi >= -70) {
        wifi_icon = epd_bmp_wifi_weak;
      } else if (rssi < -70 && rssi >= -80) {
        wifi_icon = epd_bmp_wifi_no_signal;
      } else {
        wifi_icon = epd_bmp_wifi_no_signal;
      }
      // Clear and draw the new WiFi icon, based on RSSI
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
      @note   This method overrides the base class method to provide specific
              functionality for the Think Ink Grayscale 4 EAAMGFGN driver.
  */
  virtual void writeMessage(const char *message) override {
    if (_display == nullptr)
      return;

    // Clear only the area below the status bar
    _display->fillRect(0, STATUS_BAR_HEIGHT, _display->width(),
                       _display->height() - STATUS_BAR_HEIGHT, EPD_WHITE);
    // Add padding between status bar and text content
    int16_t y_idx = STATUS_BAR_HEIGHT + 4;
    _display->setCursor(0, y_idx);

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);

    // Reset the text size to the configured value before we write
    _display->setTextSize(_text_sz);

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
        _display->write(char(247));
        i++;
      } else {
        _display->print(message[i]);
      }
    }
    _display->display();
  }

private:
  ThinkInk_290_Grayscale4_T5 *_display;
};

#endif // WS_DRV_THINKINK_GRAYSCALE4_T5_H