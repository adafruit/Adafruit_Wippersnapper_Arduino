/*!
 * @file src/components/display/drivers/dispDrvSt7789.h
 *
 * Driver for ST7789-based TFT displays.
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
#ifndef WS_DISP_DRV_ST7789
#define WS_DISP_DRV_ST7789

#include "dispDrvBase.h"
#include <Adafruit_ST7789.h>

#define ST7789_TEXT_SZ_DEFAULT 2    ///< Default text size for ST7789 displays
#define ST7789_STATUSBAR_HEIGHT 20  ///< Default status bar height
#define ST7789_STATUSBAR_ICON_SZ 16 ///< Default status bar icon size
#define ST7789_STATUSBAR_ICON_SPACING                                          \
  4 ///< Default spacing between status bar icons
#define ST7789_STATUSBAR_ICON_MARGIN                                           \
  5 ///< Default margin from edge of display to status bar icons

/*!
    @brief  Driver for ST7789-based TFT displays.
*/
class dispDrvSt7789 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the ST7789 display driver.
      @param  cs
              Chip Select pin for the display.
      @param  dc
              Data/Command pin for the display.
      @param  mosi
              MOSI pin for the display.
      @param  sck
              SCK pin for the display.
      @param  rst
              Optional Reset pin for the display.
      @param  miso
              Optional MISO pin for the display.
  */
  dispDrvSt7789(int16_t cs, int16_t dc, int16_t mosi, int16_t sck,
                int16_t rst = -1, int16_t miso = -1)
      : dispDrvBase(cs, dc, mosi, sck, rst, miso), _display(nullptr) {}

  /*!
      @brief  Destructor for the ST7789 display driver.
  */
  ~dispDrvSt7789() {
    if (_display) {
      delete _display;
      _display = nullptr;
    }
// Turn off backlight
#if defined(ARDUINO_FUNHOUSE_ESP32S2)
    digitalWrite(TFT_BACKLIGHT, LOW);
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT) ||                      \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT) ||                           \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
    digitalWrite(TFT_BACKLITE, LOW);
#endif
  }

  /*!
      @brief  Attempts to initialize the ST7789 TFT driver.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin() override {

    _display = new Adafruit_ST7789(_pin_cs, _pin_dc, _pin_rst);
    if (!_display)
      return false;

    _display->init(_width, _height);
    _display->setRotation(_rotation);
    setTextSize(ST7789_TEXT_SZ_DEFAULT);
    _display->fillScreen(ST77XX_BLACK);
    _display->setTextColor(ST77XX_WHITE);

    // Turn on backlight
#if defined(ARDUINO_FUNHOUSE_ESP32S2)
    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, HIGH);
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT) ||                      \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT) ||                           \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
#endif

    return true;
  }

  /*!
      @brief  Sets the text size for the display.
      @param  s
              The text size to set.
      @note   This method overrides the base class method to provide specific
              functionality for the ST7789 driver.
  */
  void setTextSize(uint8_t s) override {
    if (!_display)
      return;
    _text_sz = s;
    _display->setTextSize(s);
  }

  /*!
      @brief  Displays the splash screen on the display.
  */
  void showSplash() override {
    if (!_display)
      return;

    // Display the appropriate splash screen based on resolution
    if (_width == 240 && _height == 240) {
      _display->drawBitmap(0, 0, tft_bmp_logo_240240, 240, 240, ST77XX_WHITE);
    } else if (_width == 135 && _height == 240) {
      _display->drawBitmap(0, 0, tft_bmp_logo_240135, 240, 135, ST77XX_WHITE);
    } else {
      // Unsupported resolution
      return;
    }

    delay(500);
  }

  /*!
      @brief  Draws a status bar at the top of the display.
  */
  virtual void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;

    // Clear the entire display buffer to remove splash screen
    _display->fillScreen(ST77XX_BLACK);

    // Draw status bar
    _display->fillRect(0, 0, _display->width(), ST7789_STATUSBAR_HEIGHT,
                       ST77XX_WHITE);

    // Draw username on left side of the status bar
    _display->setTextSize(1);
    _display->setTextColor(ST77XX_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);

    // Calculate icon positions (rightmost side of status bar), center
    // vertically
    _statusbar_icons_y = (ST7789_STATUSBAR_HEIGHT - ST7789_STATUSBAR_ICON_SZ) / 2;
    _statusbar_icon_battery_x = _display->width() - ST7789_STATUSBAR_ICON_SZ -
                   ST7789_STATUSBAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x - ST7789_STATUSBAR_ICON_SZ - ST7789_STATUSBAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x - ST7789_STATUSBAR_ICON_SZ - ST7789_STATUSBAR_ICON_SPACING;
    // Draw icons
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y, epd_bmp_cloud_online,
                         ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                         ST77XX_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y, epd_bmp_wifi_full,
                         ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                         ST77XX_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y, epd_bmp_bat_full,
                         ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                         ST77XX_BLACK);
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

    // Only update wifi icon if the RSSI has changed significantly (+/-3dB)
    bool update_rssi = abs(rssi - _statusbar_rssi) >= 3;
    // Only update cloud icon if MQTT status has changed
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;

    // No need to update if nothing has changed
    if (!update_rssi && !update_mqtt)
      return;

    if (update_mqtt) {
      // Clear and draw the new cloud icon, based on MQTT connection status
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y, ST7789_STATUSBAR_ICON_SZ,
                         ST7789_STATUSBAR_ICON_SZ, ST77XX_WHITE);
      if (mqtt_status) {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y, epd_bmp_cloud_online,
                             ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                             ST77XX_BLACK);
      } else {
        _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y, epd_bmp_cloud_offline,
                             ST7789_STATUSBAR_ICON_SZ, ST7789_STATUSBAR_ICON_SZ,
                             ST77XX_BLACK);
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
      _display->fillRect(_statusbar_icon_wifi_x, _statusbar_icons_y, ST7789_STATUSBAR_ICON_SZ,
                         ST7789_STATUSBAR_ICON_SZ, ST77XX_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y, wifi_icon, ST7789_STATUSBAR_ICON_SZ,
                           ST7789_STATUSBAR_ICON_SZ, ST77XX_BLACK);
      _statusbar_rssi = rssi;
    }
  }

  /*!
      @brief  Writes a message to the display.
      @param  message
              The message to write to the display.
      @note   This method overrides the base class method to provide specific
              functionality for the ST7789 driver.
  */
  virtual void writeMessage(const char *message) override {
    if (_display == nullptr)
      return;

    _display->setTextColor(ST77XX_WHITE);

    // Clear only the area below the status bar
    _display->fillRect(0, ST7789_STATUSBAR_HEIGHT, _width,
                       _height - ST7789_STATUSBAR_HEIGHT, ST77XX_BLACK);
    int16_t y_idx = ST7789_STATUSBAR_HEIGHT;

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);
    // Begin with a small offset from status bar
    y_idx += 5;
    _display->setCursor(0, y_idx);
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
        _display->write(char(248));
        i++;
      } else {
        _display->print(message[i]);
      }
    }
  }

private:
  Adafruit_ST7789 *_display;
};

#endif // WS_DISP_DRV_ST7789