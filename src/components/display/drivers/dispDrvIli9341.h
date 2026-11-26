/*!
 * @file src/components/display/drivers/dispDrvIli9341.h
 *
 * Driver for ILI9341-based TFT displays (V2), e.g. the 2.2" 320x240 TFT
 * (ADA1480).
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
#ifndef WS_DISP_DRV_ILI9341_H
#define WS_DISP_DRV_ILI9341_H

#include "dispDrvBase.h"
#include <Adafruit_ILI9341.h>

#define ILI9341_STATUSBAR_HEIGHT 20      ///< Status bar height in pixels
#define ILI9341_STATUSBAR_ICON_SZ 16     ///< Status bar icon size in pixels
#define ILI9341_STATUSBAR_ICON_SPACING 4 ///< Spacing between icons
#define ILI9341_STATUSBAR_ICON_MARGIN 5  ///< Margin from edge of display

/*!
    @brief  Driver for ILI9341-based TFT displays.
*/
class dispDrvIli9341 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the ILI9341 TFT driver.
      @param  cs    Chip Select pin.
      @param  dc    Data/Command pin.
      @param  mosi  MOSI pin.
      @param  sck   SCK pin.
      @param  rst   Reset pin (-1 if not used).
      @param  miso  MISO pin (-1 if not used).
  */
  dispDrvIli9341(int16_t cs, int16_t dc, int16_t mosi, int16_t sck,
                 int16_t rst = -1, int16_t miso = -1)
      : dispDrvBase(cs, dc, mosi, sck, rst, miso), _display(nullptr) {}

  /*!
      @brief  Destructor. Blanks the panel, frees it and turns the backlight
              off.
  */
  ~dispDrvIli9341() {
    if (_display) {
      _display->fillScreen(ILI9341_BLACK);
      delete _display;
      _display = nullptr;
    }
    if (_pin_bl >= 0)
      digitalWrite(_pin_bl, _bl_active_low ? HIGH : LOW);
  }

  /*!
      @brief  Initializes the ILI9341 TFT display.
      @return True if initialization succeeded, False otherwise.
  */
  bool begin() override {
    _display =
        new Adafruit_ILI9341(_pin_cs, _pin_dc, _pin_mosi, _pin_sck, _pin_rst);
    if (!_display)
      return false;

    // Native 240x320 panel; rotation comes from the Add message
    _display->begin();
    _display->setRotation(_rotation);
    _display->fillScreen(ILI9341_BLACK);
    _display->setTextColor(ILI9341_WHITE);
    _display->setTextWrap(false);
    _display->setTextSize(_text_sz);

    // Turn on backlight (honor active-low panels)
    if (_pin_bl >= 0) {
      pinMode(_pin_bl, OUTPUT);
      digitalWrite(_pin_bl, _bl_active_low ? LOW : HIGH);
      WS_DEBUG_PRINTLN("[display] Backlight ON");
    }

    WS_DEBUG_PRINTLN("[display] ILI9341 initialized");
    return true;
  }

  /*!
      @brief  Draws the status bar and the Adafruit IO username.
      @param  io_username  Adafruit IO username to display.
  */
  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;

    // Clear entire display to remove splash screen
    _display->fillScreen(ILI9341_BLACK);

    // Draw white status bar at top
    _display->fillRect(0, 0, _display->width(), ILI9341_STATUSBAR_HEIGHT,
                       ILI9341_WHITE);

    // Draw username on left side
    _display->setTextSize(1);
    _display->setTextColor(ILI9341_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);

    // Calculate icon positions (right-aligned), centered vertically
    _statusbar_icons_y =
        (ILI9341_STATUSBAR_HEIGHT - ILI9341_STATUSBAR_ICON_SZ) / 2;
    _statusbar_icon_battery_x = _display->width() - ILI9341_STATUSBAR_ICON_SZ -
                                ILI9341_STATUSBAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x -
                             ILI9341_STATUSBAR_ICON_SZ -
                             ILI9341_STATUSBAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x -
                              ILI9341_STATUSBAR_ICON_SZ -
                              ILI9341_STATUSBAR_ICON_SPACING;

    // Draw icons
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, ILI9341_STATUSBAR_ICON_SZ,
                         ILI9341_STATUSBAR_ICON_SZ, ILI9341_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, ILI9341_STATUSBAR_ICON_SZ,
                         ILI9341_STATUSBAR_ICON_SZ, ILI9341_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, ILI9341_STATUSBAR_ICON_SZ,
                         ILI9341_STATUSBAR_ICON_SZ, ILI9341_BLACK);

    // Reset text color and size for main text area
    _display->setTextColor(ILI9341_WHITE);
    _display->setTextSize(_text_sz);
  }

  /*!
      @brief  Updates the status bar icons based on current state.
      @param  rssi         Current WiFi RSSI value.
      @param  bat          Current battery level (0-100).
      @param  mqtt_status  True if MQTT is connected.
  */
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    if (!_display)
      return;

    bool update_rssi = abs(rssi - _statusbar_rssi) >= 3;
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;

    if (!update_rssi && !update_mqtt)
      return;

    if (update_mqtt) {
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         ILI9341_STATUSBAR_ICON_SZ, ILI9341_STATUSBAR_ICON_SZ,
                         ILI9341_WHITE);
      if (mqtt_status) {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_online, ILI9341_STATUSBAR_ICON_SZ,
                             ILI9341_STATUSBAR_ICON_SZ, ILI9341_BLACK);
      } else {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_offline, ILI9341_STATUSBAR_ICON_SZ,
                             ILI9341_STATUSBAR_ICON_SZ, ILI9341_BLACK);
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
                         ILI9341_STATUSBAR_ICON_SZ, ILI9341_STATUSBAR_ICON_SZ,
                         ILI9341_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, ILI9341_STATUSBAR_ICON_SZ,
                           ILI9341_STATUSBAR_ICON_SZ, ILI9341_BLACK);
      _statusbar_rssi = rssi;
    }
  }

  /*!
      @brief  Writes a message below the status bar.
      @param  message  Message text to render.
  */
  void writeMessage(const char *message) override {
    if (!_display)
      return;

    // Clear only below status bar, start text below status bar
    _display->fillRect(0, ILI9341_STATUSBAR_HEIGHT, _display->width(),
                       _display->height() - ILI9341_STATUSBAR_HEIGHT,
                       ILI9341_BLACK);

    int16_t y_idx = ILI9341_STATUSBAR_HEIGHT + 5;
    int16_t line_height = 8 * _text_sz;

    _display->setTextSize(_text_sz);
    _display->setCursor(0, y_idx);

    size_t msg_len = strlen(message);
    for (size_t i = 0; i < msg_len; i++) {
      // Stop if we'd overflow the display
      if (y_idx + line_height > _display->height())
        break;

      char parsed_char = 0;
      bool is_newline = false;
      if (parseWriteToken(message, msg_len, i, parsed_char, is_newline,
                          char(247))) {
        if (is_newline) {
          y_idx += line_height;
          if (y_idx + line_height > _display->height())
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
  Adafruit_ILI9341 *_display; ///< ILI9341 driver instance
};

#endif // WS_DISP_DRV_ILI9341_H
