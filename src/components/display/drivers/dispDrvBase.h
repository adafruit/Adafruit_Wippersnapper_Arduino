/*!
 * @file src/components/display/drivers/dispDrvBase.h
 *
 * Abstract base class for display drivers (V2).
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
#ifndef WS_DISP_DRV_BASE_H
#define WS_DISP_DRV_BASE_H

#include "../assets/icons.h"
#include "../assets/splash.h"
#include "Adafruit_ThinkInk.h"
#include "Wippersnapper.h"

// Shared status bar constants for EPD drivers
#define STATUS_BAR_HEIGHT 20  ///< Height of the status bar in pixels
#define STATUS_BAR_BORDER 1   ///< Border around the status bar in pixels
#define STATUS_BAR_ICON_SZ 16 ///< Size of status bar icons in pixels
#define STATUS_BAR_ICON_SPACING                                                \
  4 ///< Spacing between status bar icons in pixels
#define STATUS_BAR_ICON_MARGIN                                                 \
  5 ///< Margin from edge of display to status bar icons in pixels

/*!
    @brief  Abstract base class for display drivers.
*/
class dispDrvBase {
public:
  /*!
      @brief  Constructor for displays with hardwired pins (e.g., Qualia RGB666).
  */
  dispDrvBase() {}

  /*!
      @brief  Constructor for SPI EPD (E-Ink) displays.
  */
  dispDrvBase(int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs = -1,
              int16_t busy = -1)
      : _pin_dc(dc), _pin_rst(rst), _pin_cs(cs), _pin_sram_cs(sram_cs),
        _pin_busy(busy) {}

  /*!
      @brief  Constructor for SPI TFT displays.
  */
  dispDrvBase(int16_t cs, int16_t dc, int16_t mosi, int16_t sck,
              int16_t rst, int16_t miso)
      : _pin_cs(cs), _pin_dc(dc), _pin_mosi(mosi), _pin_sck(sck),
        _pin_rst(rst), _pin_miso(miso) {}

  virtual ~dispDrvBase() {}

  /// Attempts to initialize a TFT display.
  virtual bool begin() { return false; }

  /// Attempts to initialize an EPD display with a ThinkInk mode.
  virtual bool begin(thinkinkmode_t mode, bool reset = true) { return false; }

  /// Writes a message to the display.
  virtual void writeMessage(const char *message, bool clear_first = true,
                            int32_t cursor_x = 0, int32_t cursor_y = 0) = 0;

  void setWidth(int16_t w) { _width = w; }
  void setHeight(int16_t h) { _height = h; }
  void setRotation(uint8_t r) { _rotation = r; }
  virtual void setTextSize(uint8_t s) { _text_sz = s; }
  // TODO: Move backlight pin into proto Add message instead of board defines
  void setBacklightPin(int16_t pin) { _pin_bl = pin; }

  virtual void showSplash() {}
  virtual void drawStatusBar(const char *io_username) {}
  virtual void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) {}

protected:
  int16_t _pin_cs;          ///< Chip Select pin
  int16_t _pin_dc;          ///< Data/Command pin
  int16_t _pin_mosi = -1;   ///< MOSI pin (TFT only)
  int16_t _pin_sck = -1;    ///< SCK pin (TFT only)
  int16_t _pin_rst;         ///< Reset pin
  int16_t _pin_miso = -1;   ///< MISO pin (TFT only)
  int16_t _pin_sram_cs = -1; ///< SRAM Chip Select pin (EPD only)
  int16_t _pin_busy = -1;   ///< Busy pin (EPD only)
  int16_t _pin_bl = -1;     ///< Backlight pin (-1 = not set)
  uint8_t _text_sz = 1;     ///< Text size multiplier
  int16_t _width;            ///< Display width
  int16_t _height;           ///< Display height
  uint8_t _rotation;         ///< Display rotation (0-3)
  // Status bar properties
  int _statusbar_icons_y;        ///< Y position of status bar icons
  int _statusbar_icon_battery_x; ///< X position of battery icon
  int _statusbar_icon_wifi_x;    ///< X position of WiFi icon
  int _statusbar_icon_cloud_x;   ///< X position of cloud icon
  int8_t _statusbar_rssi;        ///< Last RSSI value
  uint8_t _statusbar_bat;        ///< Last battery level percentage
  bool _statusbar_mqtt_connected; ///< Last MQTT connection status
};

#endif // WS_DISP_DRV_BASE_H
