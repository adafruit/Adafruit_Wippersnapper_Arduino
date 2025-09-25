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

#define STATUS_BAR_HEIGHT 20 ///< Height of the status bar in pixels, assumes 16px icons
#define STATUS_BAR_BORDER 1 ///< Border around the status bar in pixels
#define STATUS_BAR_ICON_SZ 16 ///< Size of status bar icons in pixels

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
    _display->clearBuffer();
    return true;
  }

  /*!
      @brief  Displays a splash screen
  */
  virtual void showSplash() override {
    if (!_display)
      return;
    _display->drawBitmap(0, 0, epd_bitmap_ws_logo_296128, 296, 128, EPD_BLACK);
    _display->display();
  }

  /*!
      @brief  Draws a status bar at the top of the display.
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

    // Calculate icon positions and center vertically
    int iconSpacing = 4;
    int rightMargin = 5;
    int iconY = STATUS_BAR_BORDER + ((STATUS_BAR_HEIGHT - 2 * STATUS_BAR_BORDER - STATUS_BAR_ICON_SZ) / 2);
    int batteryX = _display->width() - STATUS_BAR_ICON_SZ - rightMargin;
    int wifiX = batteryX - STATUS_BAR_ICON_SZ - iconSpacing;
    int cloudX = wifiX - STATUS_BAR_ICON_SZ - iconSpacing;
    // Draw icons on right side of the status bar
    _display->drawBitmap(cloudX, iconY, epd_bmp_cloud_online, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(wifiX, iconY, epd_bmp_wifi_full, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(batteryX, iconY, epd_bmp_bat_full, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);

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

  bool do_update = false;
  // Update cloud icon only if it changed
  if (mqtt_status != _statusbar_mqtt_connected) {
    int iconSpacing = 4;
    int rightMargin = 5;
    int iconY = STATUS_BAR_BORDER + ((STATUS_BAR_HEIGHT - 2 * STATUS_BAR_BORDER - STATUS_BAR_ICON_SZ) / 2);
    int batteryX = _display->width() - STATUS_BAR_ICON_SZ - rightMargin;
    int wifiX = batteryX - STATUS_BAR_ICON_SZ - iconSpacing;
    int cloudX = wifiX - STATUS_BAR_ICON_SZ - iconSpacing;

    // Clear and draw the new cloud icon, based on MQTT connection status
    _display->fillRect(cloudX, iconY, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_WHITE);
    if (mqtt_status == 21) {
      _display->drawBitmap(cloudX, iconY, epd_bmp_cloud_online, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ,
                           EPD_BLACK);
    } else {
      _display->drawBitmap(cloudX, iconY, epd_bmp_cloud_offline, STATUS_BAR_ICON_SZ,
                           STATUS_BAR_ICON_SZ, EPD_BLACK);
    }
    _statusbar_mqtt_connected = mqtt_status;
    do_update = true;
  }

/*     // Update WiFi icon only if it changed significantly (+/- 3 dB)
    if (abs(rssi - _statusbar_rssi) >= 3 || rssi == 0 || _statusbar_rssi == 0) {
      int iconSpacing = 4;
      int rightMargin = 5;
      int iconY = STATUS_BAR_BORDER + ((STATUS_BAR_HEIGHT - 2 * STATUS_BAR_BORDER - STATUS_BAR_ICON_SZ) / 2);
      int batteryX = _display->width() - STATUS_BAR_ICON_SZ - rightMargin;
      int wifiX = batteryX - STATUS_BAR_ICON_SZ - iconSpacing;

      // Clear and draw the new WiFi icon, based on RSSI
      _display->fillRect(wifiX, iconY, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_WHITE);
      if (rssi == 0) {
        _display->drawBitmap(wifiX, iconY, epd_bmp_wifi_no_signal, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
      } else if (rssi > -50) {
        _display->drawBitmap(wifiX, iconY, epd_bmp_wifi_full, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
      } else if (rssi > -60) {
        _display->drawBitmap(wifiX, iconY, epd_bmp_wifi_fair, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
      } else if (rssi > -70) {
        _display->drawBitmap(wifiX, iconY, epd_bmp_wifi_weak, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
      } else {
        _display->drawBitmap(wifiX, iconY, epd_bmp_wifi_weak, STATUS_BAR_ICON_SZ, STATUS_BAR_ICON_SZ, EPD_BLACK);
      }
      _statusbar_rssi = rssi;
      do_update = true;
    } */

    // Temporarily removed while I get clarification on how often to refresh the epd
  /*   if (do_update)
      _display->display(); */

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