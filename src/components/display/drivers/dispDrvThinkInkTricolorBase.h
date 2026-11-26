/*!
 * @file src/components/display/drivers/dispDrvThinkInkTricolorBase.h
 *
 * Shared driver logic for ThinkInk tri-colour eInk panels (Z10, Z13, Z17,
 * Z90, Z94). Each panel subclass only creates its ThinkInk panel object.
 *
 * Tri-colour panels take about 13-15 s per full refresh, so the status bar
 * is refreshed at most every 5 minutes, and a message write redraws the
 * status bar icons in the same refresh.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DRV_THINKINK_TRICOLOR_BASE_H
#define WS_DRV_THINKINK_TRICOLOR_BASE_H

#include "dispDrvBase.h"

/*!
    @brief  Base driver for ThinkInk tri-colour eInk panels.
*/
class dispDrvThinkInkTricolorBase : public dispDrvBase {
public:
  /*!
      @brief  Constructor for a ThinkInk tri-colour driver.
      @param  dc       Data/Command pin.
      @param  rst      Reset pin.
      @param  cs       Chip Select pin.
      @param  sram_cs  Optional SRAM Chip Select pin (-1 for none).
      @param  busy     Optional Busy pin (-1 for none).
  */
  dispDrvThinkInkTricolorBase(int16_t dc, int16_t rst, int16_t cs,
                              int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  /*!
      @brief  Destructor. Clears the panel and frees the ThinkInk display.
  */
  virtual ~dispDrvThinkInkTricolorBase() {
    if (_display) {
      _display->clearBuffer();
      _display->display();
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Initializes the display with the given ThinkInk mode.
      @param  mode   The ThinkInk mode to use.
      @param  reset  True to perform a hardware reset (unused here).
      @return True if initialization succeeded, False otherwise.
  */
  bool begin(thinkinkmode_t mode, bool reset = true) override {
    (void)reset;
    _display = createPanel(mode);
    if (!_display)
      return false;
    _display->setTextSize(_text_sz);
    _display->setTextColor(EPD_BLACK);
    _display->setTextWrap(false);
    _height = _display->height();
    _width = _display->width();
    _display->clearBuffer();
    return true;
  }

  /*!
      @brief  Draws the status bar and the Adafruit IO username.
      @param  io_username  Adafruit IO username to display.
  */
  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;
    _display->clearBuffer();
    _display->fillRect(0, 0, _display->width(), EPD_STATUS_BAR_HEIGHT,
                       EPD_BLACK);
    _display->fillRect(EPD_STATUS_BAR_BORDER, EPD_STATUS_BAR_BORDER,
                       _display->width() - (2 * EPD_STATUS_BAR_BORDER),
                       EPD_STATUS_BAR_HEIGHT - (2 * EPD_STATUS_BAR_BORDER),
                       EPD_WHITE);
    _display->setTextSize(1);
    _display->setTextColor(EPD_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);
    _statusbar_icons_y = EPD_STATUS_BAR_BORDER +
                         ((EPD_STATUS_BAR_HEIGHT - 2 * EPD_STATUS_BAR_BORDER -
                           EPD_STATUS_BAR_ICON_SZ) /
                          2);
    _statusbar_icon_battery_x =
        _display->width() - EPD_STATUS_BAR_ICON_SZ - EPD_STATUS_BAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x -
                             EPD_STATUS_BAR_ICON_SZ -
                             EPD_STATUS_BAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x - EPD_STATUS_BAR_ICON_SZ -
                              EPD_STATUS_BAR_ICON_SPACING;
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    // Match the cached state to the icons just drawn (full WiFi, online)
    _statusbar_rssi = 0;
    _statusbar_mqtt_connected = true;
    refresh();
  }

  /*!
      @brief  Records the latest status and refreshes the status bar icons,
              at most every 5 minutes since the last panel refresh (a
              message write also counts, as it redraws the icons too).
      @param  rssi         Current WiFi RSSI value.
      @param  bat          Current battery level (0-100, unused here).
      @param  mqtt_status  True if MQTT is connected.
  */
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    (void)bat;
    _latest_rssi = rssi;
    _latest_mqtt = mqtt_status;
    _have_latest = true;
    if (!_display || millis() - _last_refresh_ms < kStatusBarMinIntervalMs)
      return;
    if (drawStatusIcons(rssi, mqtt_status))
      refresh();
  }

  /*!
      @brief  Writes a message below the status bar, redraws the status bar
              icons with the latest status, and refreshes the panel once.
      @param  message  Message text to render.
  */
  void writeMessage(const char *message) override {
    if (!_display)
      return;
    _display->fillRect(0, EPD_STATUS_BAR_HEIGHT, _display->width(),
                       _display->height() - EPD_STATUS_BAR_HEIGHT, EPD_WHITE);
    int16_t y_idx = EPD_STATUS_BAR_HEIGHT + 4;
    _display->setCursor(0, y_idx);
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
    if (_have_latest)
      drawStatusIcons(_latest_rssi, _latest_mqtt);
    refresh();
  }

protected:
  /*!
      @brief  Creates and begins the concrete ThinkInk panel.
      @param  mode  The ThinkInk mode to begin the panel with.
      @return The new panel, or nullptr on failure.
  */
  virtual Adafruit_EPD *createPanel(thinkinkmode_t mode) = 0;

private:
  /// Minimum time between status bar refreshes (a full tri-colour refresh
  /// blocks for ~13-15 s).
  static const unsigned long kStatusBarMinIntervalMs = 5UL * 60UL * 1000UL;

  /*!
      @brief  Redraws the WiFi / cloud icons into the buffer when they
              changed (WiFi by 5 dBm or more). Does not refresh the panel.
      @param  rssi         WiFi RSSI value.
      @param  mqtt_status  True if MQTT is connected.
      @return True if any icon was redrawn.
  */
  bool drawStatusIcons(int8_t rssi, bool mqtt_status) {
    bool update_rssi = abs(rssi - _statusbar_rssi) >= 5;
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;
    if (update_mqtt) {
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ,
                         EPD_WHITE);
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
                         EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ,
                         EPD_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, EPD_STATUS_BAR_ICON_SZ,
                           EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
      _statusbar_rssi = rssi;
    }
    return update_rssi || update_mqtt;
  }

  /*!
      @brief  Refreshes the panel and records when, for the status bar
              rate limit.
  */
  void refresh() {
    _display->display();
    _last_refresh_ms = millis();
  }

  Adafruit_EPD *_display;             ///< ThinkInk panel instance
  unsigned long _last_refresh_ms = 0; ///< millis() of the last refresh
  int8_t _latest_rssi = 0;            ///< Latest RSSI from updateStatusBar
  bool _latest_mqtt = false;          ///< Latest MQTT state
  bool _have_latest = false;          ///< True once a status has arrived
};

#endif // WS_DRV_THINKINK_TRICOLOR_BASE_H
