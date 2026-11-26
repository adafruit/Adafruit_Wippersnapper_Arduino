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

#define RGB666_STATUSBAR_HEIGHT 20      ///< Status bar height in pixels
#define RGB666_STATUSBAR_ICON_SZ 16     ///< Status bar icon size in pixels
#define RGB666_STATUSBAR_ICON_SPACING 4 ///< Spacing between icons
#define RGB666_STATUSBAR_ICON_MARGIN 5  ///< Margin from edge of display

/*!
    @brief  Driver for RGB666 dotclock displays on Qualia ESP32-S3.
    @note   The GFX library objects behind the panel (Arduino_ESP32RGBPanel,
            Arduino_RGB_Display, Arduino_XCA9554SWSPI) have no destructors,
            and Arduino_ESP32RGBPanel keeps the esp_lcd RGB panel it creates
            (PSRAM framebuffer plus the ESP32-S3's only RGB panel slot) in a
            private handle that it never deletes. So they are created once,
            on the first successful begin(), and kept for the life of the
            firmware: re-creating them leaked them, and the second
            esp_lcd_new_rgb_panel() fails inside the library's
            ESP_ERROR_CHECK, aborting the board. A re-add of the same panel
            and size reuses them; a different panel or size needs a reboot.
            One driver instance owns the panel at a time.
*/
class dispDrvRgb666 : public dispDrvBaseRgb666 {
public:
  /*!
      @brief  Constructor — panel selection by name string.
      @param  panel  Panel identifier (e.g. "TL021WVC02", "TL032FWV01").
  */
  dispDrvRgb666(const char *panel)
      : dispDrvBaseRgb666("dispDrvRgb666", panel), _display(nullptr) {
    strncpy(_panel, panel, sizeof(_panel) - 1);
    _panel[sizeof(_panel) - 1] = '\0';
  }

  /*!
      @brief  Destructor. Blanks the panel and switches the backlight off,
              but keeps the shared GFX objects (see the class note).
  */
  ~dispDrvRgb666() {
    if (s_owner == this) {
      if (_display && s_ready) {
        _display->fillScreen(RGB565_BLACK);
        s_expander->digitalWrite(PCA_TFT_BACKLIGHT, LOW);
      }
      s_owner = nullptr;
    }
    _display = nullptr;
  }

  /*!
      @brief  Brings the panel up: creates the GFX objects on the first call,
              reuses them afterwards (see the class note).
      @return True if the panel is ready, False otherwise.
  */
  bool begin() override {
    int8_t panel_id = panelId(_panel);
    if (panel_id < 0) {
      WS_DEBUG_PRINT("[display] ERROR: Unknown RGB666 panel: ");
      WS_DEBUG_PRINTLNVAR(_panel);
      return false;
    }
    if (s_owner != nullptr && s_owner != this) {
      WS_DEBUG_PRINTLN("[display] ERROR: RGB666 panel already in use by "
                       "another display component!");
      return false;
    }

    if (s_rgbpanel != nullptr) {
      // The panel hardware was already set up by an earlier add.
      if (!s_ready) {
        WS_DEBUG_PRINTLN("[display] ERROR: RGB666 panel failed to initialize "
                         "earlier, reboot to retry!");
        return false;
      }
      if (panel_id != s_panel_id || _width != s_width || _height != s_height) {
        WS_DEBUG_PRINTLN("[display] ERROR: Changing the RGB666 panel type or "
                         "size needs a reboot!");
        return false;
      }
      _display = s_display;
      _display->setRotation(_rotation);
    } else {
      if (!createPanel(panel_id))
        return false;
      _display = s_display;
    }
    s_owner = this;

    _display->fillScreen(RGB565_BLACK);
    _display->setTextColor(RGB565_WHITE);
    _display->setTextWrap(false);
    _display->setTextSize(_text_sz);

    // Enable backlight via IO expander
    s_expander->pinMode(PCA_TFT_BACKLIGHT, OUTPUT);
    s_expander->digitalWrite(PCA_TFT_BACKLIGHT, HIGH);

    WS_DEBUG_PRINT("[display] RGB666 panel '");
    WS_DEBUG_PRINTVAR(_panel);
    WS_DEBUG_PRINTLN("' initialized");
    return true;
  }

  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;

    _display->fillScreen(RGB565_BLACK);

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
                         RGB666_STATUSBAR_ICON_SZ, RGB565_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, RGB666_STATUSBAR_ICON_SZ,
                         RGB666_STATUSBAR_ICON_SZ, RGB565_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, RGB666_STATUSBAR_ICON_SZ,
                         RGB666_STATUSBAR_ICON_SZ, RGB565_BLACK);

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

  void writeMessage(const char *message) override {
    if (!_display)
      return;

    // Clear only below status bar, start text below status bar
    _display->fillRect(0, RGB666_STATUSBAR_HEIGHT, _display->width(),
                       _display->height() - RGB666_STATUSBAR_HEIGHT,
                       RGB565_BLACK);

    int16_t y_idx = RGB666_STATUSBAR_HEIGHT + 5;
    int16_t line_height = 8 * _text_sz;

    _display->setTextSize(_text_sz);
    _display->setCursor(0, y_idx);

    size_t msg_len = strlen(message);
    for (size_t i = 0; i < msg_len; i++) {
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
  /*!
      @brief  Maps a panel name to a panel type.
      @param  panel  Panel part number or Adafruit PID.
      @return 0 for the 2.1" round 480x480, 1 for the 3.2" bar 320x820, or
              -1 if the name is unknown.
  */
  static int8_t panelId(const char *panel) {
    if (strcmp(panel, "TL021WVC02") == 0 || strcmp(panel, "adafruit-5792") == 0)
      return 0;
    if (strcmp(panel, "TL032FWV01") == 0 || strcmp(panel, "adafruit-5797") == 0)
      return 1;
    return -1;
  }

  /*!
      @brief  Creates the shared GFX objects for a panel type and begins the
              display. Runs at most once per boot: the objects are stored
              as soon as they exist, so a failure is never retried.
      @param  panel_id  Panel type from panelId().
      @return True if the display began, False otherwise.
  */
  bool createPanel(int8_t panel_id) {
    Wire.setClock(400000);

    // Arduino_XCA9554SWSPI::begin() still returns true when the expander
    // doesn't answer (every later write is then a silent no-op), which would
    // latch a dark panel as ready for the whole boot. Probe it first, before
    // anything is allocated, so a later add can retry.
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() != 0) {
      WS_DEBUG_PRINTLN("[display] ERROR: PCA9554A I/O expander (0x3F) not "
                       "found on the I2C bus!");
      return false;
    }

    // Select init operations and dotclock timings per panel.
    // Timings from CircuitPython dotclockframebuffer configs.
    const uint8_t *init_ops = nullptr;
    size_t init_ops_len = 0;
    uint16_t h_fp, h_pw, h_bp, v_fp, v_pw, v_bp;
    uint16_t pclk_active_neg = 0;
    int32_t prefer_speed = 16000000;

    if (panel_id == 0) {
      // 2.1" round 480x480
      init_ops = TL021WVC02_init_operations;
      init_ops_len = sizeof(TL021WVC02_init_operations);
      h_fp = 46;
      h_pw = 2;
      h_bp = 44;
      v_fp = 50;
      v_pw = 16;
      v_bp = 16;
      pclk_active_neg = 0; // pclk_active_high = True
    } else {
      // 3.2" bar 320x820
      init_ops = tl032fwv01_init_operations;
      init_ops_len = sizeof(tl032fwv01_init_operations);
      h_fp = 150;
      h_pw = 3;
      h_bp = 251;
      v_fp = 100;
      v_pw = 6;
      v_bp = 90;
      pclk_active_neg = 1; // pclk_active_high = False
    }

    s_expander = new Arduino_XCA9554SWSPI(
        PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI, &Wire, 0x3F);
    s_rgbpanel = new Arduino_ESP32RGBPanel(
        TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK, TFT_R1, TFT_R2, TFT_R3, TFT_R4,
        TFT_R5, TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5, TFT_B1, TFT_B2,
        TFT_B3, TFT_B4, TFT_B5, 1 /* hsync_polarity */, h_fp, h_pw, h_bp,
        1 /* vsync_polarity */, v_fp, v_pw, v_bp, pclk_active_neg,
        prefer_speed);
    s_display = new Arduino_RGB_Display(
        _width, _height, s_rgbpanel, _rotation, true /* auto_flush */,
        s_expander, GFX_NOT_DEFINED /* RST */, init_ops, init_ops_len);
    s_panel_id = panel_id;
    s_width = _width;
    s_height = _height;

    if (!s_display->begin()) {
      WS_DEBUG_PRINTLN("[display] ERROR: Arduino_RGB_Display begin() failed!");
      return false;
    }
    s_ready = true;
    return true;
  }

  Arduino_RGB_Display *_display; ///< This instance's view of s_display
  char _panel[20];               ///< Panel name from the Add message

  static inline Arduino_RGB_Display *s_display =
      nullptr; ///< Shared display, created once (see the class note)
  static inline Arduino_XCA9554SWSPI *s_expander =
      nullptr; ///< Shared I/O expander bus for the panel, created once
  static inline Arduino_ESP32RGBPanel *s_rgbpanel =
      nullptr; ///< Shared dot-clock RGB panel, created once
  static inline int8_t s_panel_id =
      -1; ///< Panel type the shared objects were made for, from panelId()
  static inline int16_t s_width = 0;  ///< Width the panel was created with
  static inline int16_t s_height = 0; ///< Height the panel was created with
  static inline bool s_ready =
      false; ///< True once the shared display has begun successfully
  static inline dispDrvRgb666 *s_owner =
      nullptr; ///< Driver instance currently using the shared panel
};

#endif // ARDUINO_ADAFRUIT_QUALIA_S3_RGB666

#endif // WS_DISP_DRV_RGB666_H
