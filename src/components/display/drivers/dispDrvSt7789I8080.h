/*!
 * @file src/components/display/drivers/dispDrvSt7789I8080.h
 *
 * Driver for ST7789-based TFT displays driven over an Intel 8080 (i8080)
 * 8-bit parallel bus, via Arduino_GFX (V2).
 *
 * Targets the LilyGo T-Display-S3 (1.9" 170x320 IPS, ESP32-S3). The ST7789
 * controller has 240x320 of RAM; the 170-wide panel is centred on the column
 * axis (column offset 35, row offset 0) and runs inverted (IPS). Panel power
 * (LCD_POWER_ON) and backlight (LCD_BL) are board-fixed and enabled here.
 *
 * Reference (offsets / inversion): CircuitPython lilygo_tdisplay_s3 board.c
 * https://github.com/adafruit/circuitpython/blob/main/ports/espressif/boards/lilygo_tdisplay_s3/board.c
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
#ifndef WS_DISP_DRV_ST7789_I8080_H
#define WS_DISP_DRV_ST7789_I8080_H

// Arduino_ESP32LCD8 (the i8080 8-bit data bus) requires the ESP32-S3 LCD_CAM
// peripheral, so this driver only compiles for the ESP32-S3.
#if defined(ESP32) && defined(CONFIG_IDF_TARGET_ESP32S3)

#include "dispDrvBase.h"
#include <Arduino_GFX_Library.h>

#define ST7789_I8080_STATUSBAR_HEIGHT 20  ///< Status bar height in pixels
#define ST7789_I8080_STATUSBAR_ICON_SZ 16 ///< Status bar icon size in pixels
#define ST7789_I8080_STATUSBAR_ICON_SPACING 4 ///< Spacing between icons
#define ST7789_I8080_STATUSBAR_ICON_MARGIN 5  ///< Margin from edge of display

/*!
    @brief  Driver for ST7789 TFT displays on an 8-bit i8080 parallel bus.
*/
class dispDrvSt7789I8080 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the i8080 ST7789 TFT driver.
      @param  dc   Data/Command pin.
      @param  cs   Chip Select pin.
      @param  wr   Write-strobe pin.
      @param  rd   Read-strobe pin (-1 / GFX_NOT_DEFINED if not used).
      @param  d0   Data bus bit 0 pin.
      @param  d1   Data bus bit 1 pin.
      @param  d2   Data bus bit 2 pin.
      @param  d3   Data bus bit 3 pin.
      @param  d4   Data bus bit 4 pin.
      @param  d5   Data bus bit 5 pin.
      @param  d6   Data bus bit 6 pin.
      @param  d7   Data bus bit 7 pin.
      @param  rst  Reset pin (-1 if not used).
  */
  dispDrvSt7789I8080(int16_t dc, int16_t cs, int16_t wr, int16_t rd, int16_t d0,
                     int16_t d1, int16_t d2, int16_t d3, int16_t d4, int16_t d5,
                     int16_t d6, int16_t d7, int16_t rst = -1)
      : dispDrvBase(), _bus(nullptr), _display(nullptr), _pin_wr(wr),
        _pin_rd(rd), _pin_power(-1) {
    _pin_dc = dc;
    _pin_cs = cs;
    _pin_rst = rst;
    _data_pins[0] = d0;
    _data_pins[1] = d1;
    _data_pins[2] = d2;
    _data_pins[3] = d3;
    _data_pins[4] = d4;
    _data_pins[5] = d5;
    _data_pins[6] = d6;
    _data_pins[7] = d7;
  }

  ~dispDrvSt7789I8080() {
    if (_display) {
      _display->fillScreen(RGB565_BLACK);
      delete _display;
      _display = nullptr;
    }
    // Arduino_GFX does not own/free the data bus, so release it explicitly.
    if (_bus) {
      delete _bus;
      _bus = nullptr;
    }
    // TODO: once the backlight routes through the digitalio/pwm controllers
    // (see begin()), release it there instead of driving the pin directly.
    if (_pin_bl >= 0)
      digitalWrite(_pin_bl, LOW);
    if (_pin_power >= 0)
      digitalWrite(_pin_power, LOW);
  }

  /*!
      @brief  Sets the panel power-enable pin, driven HIGH during begin().
      @param  pin  Pin number (-1 to disable).
  */
  void setPowerPin(int16_t pin) { _pin_power = pin; }

  /*!
      @brief  Initializes the i8080 ST7789 TFT display.
      @return True if initialization succeeded, False otherwise.
  */
  bool begin() override {
    // Enable panel power rail before touching the bus.
    if (_pin_power >= 0) {
      pinMode(_pin_power, OUTPUT);
      digitalWrite(_pin_power, HIGH);
    }

    _bus = new Arduino_ESP32LCD8(_pin_dc, _pin_cs, _pin_wr, _pin_rd,
                                 _data_pins[0], _data_pins[1], _data_pins[2],
                                 _data_pins[3], _data_pins[4], _data_pins[5],
                                 _data_pins[6], _data_pins[7]);
    if (!_bus)
      return false;

    // Arduino_GFX wants native (portrait) dimensions; rotation is applied on
    // top. The panel is centred in the controller's 240x320 RAM.
    uint16_t native_w = min(_width, _height);
    uint16_t native_h = max(_width, _height);
    uint8_t col_offset = (native_w < 240) ? (240 - native_w) / 2 : 0;
    uint8_t row_offset = (native_h < 320) ? (320 - native_h) / 2 : 0;

    _display = new Arduino_ST7789(_bus, _pin_rst, _rotation, true /* IPS */,
                                  native_w, native_h, col_offset, row_offset,
                                  col_offset, row_offset);
    if (!_display) {
      delete _bus;
      _bus = nullptr;
      return false;
    }

    if (!_display->begin()) {
      WS_DEBUG_PRINTLN(
          "[display] ERROR: Arduino_ST7789 (i8080) begin() failed!");
      delete _display;
      _display = nullptr;
      delete _bus;
      _bus = nullptr;
      return false;
    }

    _display->fillScreen(RGB565_BLACK);
    _display->setTextColor(RGB565_WHITE);
    _display->setTextWrap(false);
    _display->setTextSize(_text_sz);

    // Turn on backlight.
    // TODO: route the backlight through the digitalio/pwm controllers instead
    // of raw digitalWrite, so it is runtime-controllable (on/off + PWM
    // brightness) from the broker. The v2 proto already carries the hook:
    // ws_display_Add.backlight (ws_display_BacklightConfig, a oneof of
    // ws_digitalio_Add / ws_pwm_Add) — wire that config into the respective
    // controller and have this driver stop owning the pin.
    if (_pin_bl >= 0) {
      pinMode(_pin_bl, OUTPUT);
      digitalWrite(_pin_bl, HIGH);
      WS_DEBUG_PRINTLN("[display] Backlight ON");
    }

    WS_DEBUG_PRINTLN("[display] ST7789 (i8080) initialized");
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
    _display->fillScreen(RGB565_BLACK);

    // Draw white status bar at top
    _display->fillRect(0, 0, _display->width(), ST7789_I8080_STATUSBAR_HEIGHT,
                       RGB565_WHITE);

    // Draw username on left side
    _display->setTextSize(1);
    _display->setTextColor(RGB565_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);

    // Calculate icon positions (right-aligned), centered vertically
    _statusbar_icons_y =
        (ST7789_I8080_STATUSBAR_HEIGHT - ST7789_I8080_STATUSBAR_ICON_SZ) / 2;
    _statusbar_icon_battery_x = _display->width() -
                                ST7789_I8080_STATUSBAR_ICON_SZ -
                                ST7789_I8080_STATUSBAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x -
                             ST7789_I8080_STATUSBAR_ICON_SZ -
                             ST7789_I8080_STATUSBAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x -
                              ST7789_I8080_STATUSBAR_ICON_SZ -
                              ST7789_I8080_STATUSBAR_ICON_SPACING;

    // Draw icons
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, ST7789_I8080_STATUSBAR_ICON_SZ,
                         ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, ST7789_I8080_STATUSBAR_ICON_SZ,
                         ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, ST7789_I8080_STATUSBAR_ICON_SZ,
                         ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_BLACK);

    // Reset text color and size for main text area
    _display->setTextColor(RGB565_WHITE);
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
                         ST7789_I8080_STATUSBAR_ICON_SZ,
                         ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_WHITE);
      if (mqtt_status) {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_online,
                             ST7789_I8080_STATUSBAR_ICON_SZ,
                             ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_BLACK);
      } else {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                             epd_bmp_cloud_offline,
                             ST7789_I8080_STATUSBAR_ICON_SZ,
                             ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_BLACK);
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
                         ST7789_I8080_STATUSBAR_ICON_SZ,
                         ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, ST7789_I8080_STATUSBAR_ICON_SZ,
                           ST7789_I8080_STATUSBAR_ICON_SZ, RGB565_BLACK);
      _statusbar_rssi = rssi;
    }
  }

  void writeMessage(const char *message) override {
    if (!_display)
      return;

    // Clear only below status bar, start text below status bar
    _display->fillRect(0, ST7789_I8080_STATUSBAR_HEIGHT, _display->width(),
                       _display->height() - ST7789_I8080_STATUSBAR_HEIGHT,
                       RGB565_BLACK);

    int16_t y_idx = ST7789_I8080_STATUSBAR_HEIGHT + 5;
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
  Arduino_DataBus *_bus; ///< i8080 8-bit parallel data bus
  Arduino_GFX *_display; ///< ST7789 display via Arduino_GFX
  int16_t _pin_wr;       ///< Write-strobe pin
  int16_t _pin_rd;       ///< Read-strobe pin
  int16_t _pin_power;    ///< Panel power-enable pin (-1 = not set)
  int16_t _data_pins[8]; ///< D0..D7 data bus pins
};

#endif // ESP32 && CONFIG_IDF_TARGET_ESP32S3

#endif // WS_DISP_DRV_ST7789_I8080_H
