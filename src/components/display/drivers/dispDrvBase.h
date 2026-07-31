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
#include "Adafruit_ImageReader_EPD.h"
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
  */
  virtual void writeMessage(const char *message) = 0;

  /*!
      @brief  Draws a canvas (raw .BMP file bytes) to the display.
      @param  bmp  Pointer to the complete BMP file bytes.
      @param  len  Length of the BMP buffer, in bytes.
      @return True if drawn, False if unsupported by this driver.
  */
  virtual bool drawCanvas(const uint8_t *bmp, size_t len) { return false; }


  /*!
      @brief  Draws a marquee canvas (raw .BMP file bytes) to the display.
      @param  bmp  Pointer to the complete BMP file bytes.
      @param  len  Length of the BMP buffer, in bytes.
      @return True if drawn, False if unsupported by this driver.
  */
  virtual bool drawMarqueeEPD(const uint8_t *bmp, size_t len) {
    return false;
  }

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
      @brief  Enables or disables the status bar, from
              DisplayProperties.status_bar. When disabled, drivers write
              messages from the very top of the panel instead of reserving a
              strip for the bar.
      @param  enabled  True to reserve/draw a status bar, False for full-screen.
  */
  void setStatusBar(bool enabled) { _status_bar = enabled; }

  /*!
      @brief  Returns whether the status bar is enabled for this display.
      @return True if the status bar is enabled.
  */
  bool hasStatusBar() const { return _status_bar; }

  /*!
      @brief  Records whether the MCU cold-booted or resumed from a sleep
              cycle. Must be called before begin() to take effect.
      @param  cold  True on power-on/reset, False when waking from sleep.
  */
  void setColdBoot(bool cold) { _is_cold_boot = cold; }

  /*!
      @brief  Sets the backlight control pin.
      @param  pin  Pin number for the backlight (-1 to disable).
      @param  active_low  True if the backlight is active-low (driven LOW = on).
      @note   Move backlight pin into proto Add message instead of board
              defines.
  */
  void setBacklightPin(int16_t pin, bool active_low = false) {
    _pin_bl = pin;
    _bl_active_low = active_low;
  }

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
      @brief  Blits an in-memory BMP onto an SPI TFT panel, top-left aligned.

              The EPD path uses Adafruit_ImageReader_EPD's in-memory drawBMP()
              overload, but Adafruit_ImageReader only exposes a *filesystem*
              drawBMP() for TFTs — there is no in-memory equivalent to call.
              This decodes the uncompressed BMP3 that the marquee backend emits
              (ImageMagick `-compress none BMP3:`) and pushes it a row at a
              time, so any Adafruit_SPITFT-derived panel can render a canvas.

              Handles 1/4/8-bpp palette-indexed and 24-bpp truecolor BI_RGB
              bitmaps, bottom-up or top-down, and clips anything larger than
              the panel. The buffer arrives over the network, so every read is
              bounds-checked against len.

      @param  tft  Target panel.
      @param  bmp  Pointer to the complete BMP file bytes.
      @param  len  Length of the BMP buffer, in bytes.
      @return True if the bitmap was drawn, False if it was malformed or in an
              unsupported encoding.
  */
  bool drawMarqueeBmpToTft(Adafruit_SPITFT &tft, const uint8_t *bmp,
                           size_t len) {
    // -- Header (14-byte BITMAPFILEHEADER + 40-byte BITMAPINFOHEADER) --------
    if (bmp == nullptr || len < 54 || bmp[0] != 'B' || bmp[1] != 'M') {
      WS_DEBUG_PRINTLN("[display] ERROR: Not a BMP!");
      return false;
    }
    auto rd16 = [&](size_t o) -> uint16_t {
      return (uint16_t)bmp[o] | ((uint16_t)bmp[o + 1] << 8);
    };
    auto rd32 = [&](size_t o) -> uint32_t {
      return (uint32_t)bmp[o] | ((uint32_t)bmp[o + 1] << 8) |
             ((uint32_t)bmp[o + 2] << 16) | ((uint32_t)bmp[o + 3] << 24);
    };

    uint32_t data_off = rd32(10);
    uint32_t hdr_size = rd32(14);
    int32_t bmp_w = (int32_t)rd32(18);
    int32_t bmp_h = (int32_t)rd32(22);
    uint16_t bpp = rd16(28);
    uint32_t compression = rd32(30);
    uint32_t colors_used = rd32(46);

    if (compression != 0) { // BI_RGB only
      WS_DEBUG_PRINT("[display] ERROR: Compressed BMP unsupported, type: ");
      WS_DEBUG_PRINTLNVAR(compression);
      return false;
    }
    if (bpp != 1 && bpp != 4 && bpp != 8 && bpp != 24) {
      WS_DEBUG_PRINT("[display] ERROR: Unsupported BMP bpp: ");
      WS_DEBUG_PRINTLNVAR(bpp);
      return false;
    }
    // A negative height means the rows are stored top-down.
    bool top_down = (bmp_h < 0);
    uint32_t height = (uint32_t)(top_down ? -(int64_t)bmp_h : bmp_h);
    if (bmp_w <= 0 || height == 0 || data_off >= len) {
      WS_DEBUG_PRINTLN("[display] ERROR: Bad BMP dimensions!");
      return false;
    }
    uint32_t width = (uint32_t)bmp_w;

    // -- Palette (BGRA quads, immediately after the info header) -------------
    uint16_t palette[256];
    uint32_t n_colors = 0;
    if (bpp <= 8) {
      n_colors = colors_used ? colors_used : (1UL << bpp);
      if (n_colors > 256)
        n_colors = 256;
      size_t pal_off = 14 + hdr_size;
      if (pal_off + (size_t)n_colors * 4 > len) {
        WS_DEBUG_PRINTLN("[display] ERROR: BMP palette truncated!");
        return false;
      }
      for (uint32_t i = 0; i < n_colors; i++) {
        uint8_t b = bmp[pal_off + i * 4 + 0];
        uint8_t g = bmp[pal_off + i * 4 + 1];
        uint8_t r = bmp[pal_off + i * 4 + 2];
        palette[i] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
      }
    }

    // Rows are padded out to a 4-byte boundary.
    uint32_t row_stride = (((uint32_t)width * bpp + 31) / 32) * 4;

    // Clip to whatever the panel can actually show.
    int16_t draw_w = (int16_t)min((uint32_t)tft.width(), width);
    int16_t draw_h = (int16_t)min((uint32_t)tft.height(), height);
    if (draw_w <= 0 || draw_h <= 0)
      return false;

    uint16_t *row_px = (uint16_t *)malloc((size_t)draw_w * sizeof(uint16_t));
    if (row_px == nullptr) {
      WS_DEBUG_PRINTLN("[display] ERROR: Out of memory for canvas row!");
      return false;
    }

    tft.startWrite();
    for (int16_t y = 0; y < draw_h; y++) {
      // Bottom-up bitmaps store the last row first.
      uint32_t src_row = top_down ? (uint32_t)y : (height - 1 - (uint32_t)y);
      size_t row_off = (size_t)data_off + (size_t)src_row * row_stride;
      if (row_off + row_stride > len) {
        // Truncated payload: stop early rather than read past the buffer.
        WS_DEBUG_PRINTLN("[display] WARNING: BMP rows truncated!");
        break;
      }
      const uint8_t *src = bmp + row_off;
      for (int16_t x = 0; x < draw_w; x++) {
        uint16_t color;
        if (bpp == 24) {
          const uint8_t *px = src + (size_t)x * 3;
          color = (uint16_t)(((px[2] & 0xF8) << 8) | ((px[1] & 0xFC) << 3) |
                             (px[0] >> 3));
        } else {
          uint32_t idx;
          if (bpp == 8) {
            idx = src[x];
          } else if (bpp == 4) {
            uint8_t byte = src[x >> 1];
            idx = (x & 1) ? (byte & 0x0F) : (byte >> 4);
          } else { // 1bpp, MSB first
            uint8_t byte = src[x >> 3];
            idx = (byte >> (7 - (x & 7))) & 0x01;
          }
          color = (idx < n_colors) ? palette[idx] : 0;
        }
        row_px[x] = color;
      }
      // Window must be set before the pixels for each row.
      tft.setAddrWindow(0, y, (uint16_t)draw_w, 1);
      tft.writePixels(row_px, (uint32_t)draw_w, true, false);
    }
    tft.endWrite();
    free(row_px);
    return true;
  }

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

  int16_t _pin_cs;             ///< Chip Select pin
  int16_t _pin_dc;             ///< Data/Command pin
  int16_t _pin_mosi = -1;      ///< MOSI pin (TFT only)
  int16_t _pin_sck = -1;       ///< SCK pin (TFT only)
  int16_t _pin_rst;            ///< Reset pin
  int16_t _pin_miso = -1;      ///< MISO pin (TFT only)
  int16_t _pin_sram_cs = -1;   ///< SRAM Chip Select pin (EPD only)
  int16_t _pin_busy = -1;      ///< Busy pin (EPD only)
  int16_t _pin_bl = -1;        ///< Backlight pin (-1 = not set)
  bool _bl_active_low = false; ///< True if the backlight is active-low
  uint8_t _text_sz = 1;        ///< Text size multiplier
  int16_t _width;              ///< Display width
  int16_t _height;             ///< Display height
  uint8_t _rotation;           ///< Display rotation (0-3)
  bool _status_bar = false;    ///< True when a status bar is reserved/drawn
  bool _is_cold_boot = true;   ///< True on power-on, False when resuming from a
                               ///< sleep cycle. Lets EPD drivers skip refreshes
                               ///< that only establish a state the panel
                               ///< already holds across sleep. Defaults to True
                               ///< so an un-wired caller keeps the previous
                               ///< behaviour.

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
