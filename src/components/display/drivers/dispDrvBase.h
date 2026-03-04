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
#include "Wippersnapper.h"

/*!
    @brief  Abstract base class for display drivers.
*/
class dispDrvBase {
public:
  /*!
      @brief  Constructor for SPI TFT displays.
  */
  dispDrvBase(int16_t cs, int16_t dc, int16_t mosi, int16_t sck,
              int16_t rst = -1, int16_t miso = -1)
      : _pin_cs(cs), _pin_dc(dc), _pin_mosi(mosi), _pin_sck(sck),
        _pin_rst(rst), _pin_miso(miso) {}

  virtual ~dispDrvBase() {}

  /// Attempts to initialize the display.
  virtual bool begin() { return false; }

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
  int16_t _pin_cs;      ///< Chip Select pin
  int16_t _pin_dc;      ///< Data/Command pin
  int16_t _pin_mosi;    ///< MOSI pin
  int16_t _pin_sck;     ///< SCK pin
  int16_t _pin_rst;     ///< Reset pin
  int16_t _pin_miso;    ///< MISO pin
  int16_t _pin_bl = -1; ///< Backlight pin (-1 = not set)
  uint8_t _text_sz = 1; ///< Text size multiplier
  int16_t _width;       ///< Display width
  int16_t _height;      ///< Display height
  uint8_t _rotation;    ///< Display rotation (0-3)
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
