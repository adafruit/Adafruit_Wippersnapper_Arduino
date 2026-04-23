/*!
 * @file src/components/display/drivers/dispDrvBase.h
 *
 * Abstract base class for display drivers.
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
#include "wippersnapper.h"

/*! @brief Shared status bar constants for EPD drivers. */
#define EPD_STATUS_BAR_HEIGHT 20  ///< Height of the status bar in pixels
#define EPD_STATUS_BAR_BORDER 1   ///< Border around the status bar in pixels
#define EPD_STATUS_BAR_ICON_SZ 16 ///< Size of status bar icons in pixels
#define EPD_STATUS_BAR_ICON_SPACING                                            \
  4 ///< Spacing between status bar icons in pixels
#define EPD_STATUS_BAR_ICON_MARGIN                                             \
  5 ///< Margin from edge of display to status bar icons in pixels

/*!
    @brief  Abstract base class for display drivers.
*/
class dispDrvBase {
public:
  /*!
      @brief  Constructor for displays with hardwired pins (e.g., Qualia
     RGB666).
  */
  dispDrvBase() {}

  /*!
      @brief  Constructor for SPI EPD (E-Ink) displays.
      @param  dc       Data/Command pin.
      @param  rst      Reset pin.
      @param  cs       Chip Select pin.
      @param  sram_cs  Optional SRAM Chip Select pin (-1 for none).
      @param  busy     Optional Busy pin (-1 for none).
  */
  dispDrvBase(int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs = -1,
              int16_t busy = -1)
      : _pin_dc(dc), _pin_rst(rst), _pin_cs(cs), _pin_sram_cs(sram_cs),
        _pin_busy(busy) {}

  /*!
      @brief  Constructor for SPI TFT displays.
      @param  cs    Chip Select pin.
      @param  dc    Data/Command pin.
      @param  mosi  MOSI pin.
      @param  sck   SCK pin.
      @param  rst   Reset pin.
      @param  miso  MISO pin.
  */
  dispDrvBase(int16_t cs, int16_t dc, int16_t mosi, int16_t sck, int16_t rst,
              int16_t miso)
      : _pin_cs(cs), _pin_dc(dc), _pin_mosi(mosi), _pin_sck(sck), _pin_rst(rst),
        _pin_miso(miso) {}

  /*! @brief Virtual destructor. */
  virtual ~dispDrvBase() {}

  /*!
      @brief  Attempts to initialize a TFT display.
      @return True if successful, False otherwise.
  */
  virtual bool begin() { return false; }

  /*!
      @brief  Attempts to initialize an EPD display with a ThinkInk mode.
      @param  mode   The ThinkInk mode to initialize with.
      @param  reset  True to perform a hardware reset during init.
      @return True if successful, False otherwise.
  */
  virtual bool begin(thinkinkmode_t mode, bool reset = true) { return false; }

  /*!
      @brief  Writes a message to the display.
      @param  message      Message text to render.
      @param  clear_first  True to clear the target area before drawing.
      @param  cursor_x     Horizontal cursor position.
      @param  cursor_y     Vertical cursor position.
  */
  virtual void writeMessage(const char *message, bool clear_first = true,
                            int32_t cursor_x = 0, int32_t cursor_y = 0) = 0;

  /*!
      @brief  Sets the display width in pixels.
      @param  w  Width in pixels.
  */
  void setWidth(int16_t w) { _width = w; }
  /*!
      @brief  Sets the display height in pixels.
      @param  h  Height in pixels.
  */
  void setHeight(int16_t h) { _height = h; }
  /*!
      @brief  Sets the display rotation.
      @param  r  Rotation value (0-3).
  */
  void setRotation(uint8_t r) { _rotation = r; }
  /*!
      @brief  Sets the text size multiplier.
      @param  s  Text size multiplier.
  */
  virtual void setTextSize(uint8_t s) { _text_sz = s; }

  /*!
      @brief  Sets the backlight control pin.
      @param  pin  Pin number for the backlight (-1 to disable).
      @note   Move backlight pin into proto Add message instead of board
              defines.
  */
  void setBacklightPin(int16_t pin) { _pin_bl = pin; }

  /*! @brief Shows the display splash screen, if supported. */
  virtual void showSplash() {}
  /*!
      @brief  Draws the status bar, if supported.
      @param  io_username  Adafruit IO username to display.
  */
  virtual void drawStatusBar(const char *io_username) {}
  /*!
      @brief  Updates status bar indicators, if supported.
      @param  rssi         Current WiFi RSSI value.
      @param  bat          Current battery level (0-100).
      @param  mqtt_status  True if MQTT is connected.
  */
  virtual void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) {}

protected:
  /*!
      @brief  Parses a display-write token at the given index.
      @param  message       Input message buffer.
      @param  msg_size      Message length.
      @param  idx           Current index (updated when a multi-byte token is
     consumed).
      @param  out_char      Parsed output character when token is a glyph.
      @param  is_newline    Set true when token is a newline marker.
      @param  degree_char   Display-specific glyph for the degree symbol.
      @return True when a token was recognized and consumed.
  */
  bool parseWriteToken(const char *message, size_t msg_size, size_t &idx,
                       char &out_char, bool &is_newline,
                       char degree_char = char(247)) const {
    out_char = 0;
    is_newline = false;

    if (!message || idx >= msg_size)
      return false;

    // Handle escaped CRLF and LF sequences ("\\r\\n", "\\n").
    if (message[idx] == '\\' && idx + 1 < msg_size) {
      if (message[idx + 1] == 'r' && idx + 3 < msg_size &&
          message[idx + 2] == '\\' && message[idx + 3] == 'n') {
        idx += 3;
        is_newline = true;
        return true;
      }
      if (message[idx + 1] == 'n') {
        idx += 1;
        is_newline = true;
        return true;
      }
    }

    // Handle literal CRLF, CR, and LF.
    if (message[idx] == '\r') {
      if (idx + 1 < msg_size && message[idx + 1] == '\n') {
        idx += 1;
      }
      is_newline = true;
      return true;
    }
    if (message[idx] == '\n') {
      is_newline = true;
      return true;
    }

    // Handle UTF-8 degree symbol (0xC2 0xB0).
    if ((uint8_t)message[idx] == 0xC2 && idx + 1 < msg_size &&
        (uint8_t)message[idx + 1] == 0xB0) {
      idx += 1;
      out_char = degree_char;
      return true;
    }

    return false;
  }

  int16_t _pin_cs;           ///< Chip Select pin
  int16_t _pin_dc;           ///< Data/Command pin
  int16_t _pin_mosi = -1;    ///< MOSI pin (TFT only)
  int16_t _pin_sck = -1;     ///< SCK pin (TFT only)
  int16_t _pin_rst;          ///< Reset pin
  int16_t _pin_miso = -1;    ///< MISO pin (TFT only)
  int16_t _pin_sram_cs = -1; ///< SRAM Chip Select pin (EPD only)
  int16_t _pin_busy = -1;    ///< Busy pin (EPD only)
  int16_t _pin_bl = -1;      ///< Backlight pin (-1 = not set)
  uint8_t _text_sz = 1;      ///< Text size multiplier
  int16_t _width;            ///< Display width
  int16_t _height;           ///< Display height
  uint8_t _rotation;         ///< Display rotation (0-3)

  /*! @brief Cached status bar layout and state. */
  int _statusbar_icons_y;         ///< Y position of status bar icons
  int _statusbar_icon_battery_x;  ///< X position of battery icon
  int _statusbar_icon_wifi_x;     ///< X position of WiFi icon
  int _statusbar_icon_cloud_x;    ///< X position of cloud icon
  int8_t _statusbar_rssi;         ///< Last RSSI value
  uint8_t _statusbar_bat;         ///< Last battery level percentage
  bool _statusbar_mqtt_connected; ///< Last MQTT connection status
};

#endif // WS_DISP_DRV_BASE_H
