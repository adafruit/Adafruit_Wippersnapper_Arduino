/*!
 * @file src/components/display/drivers/dispDrvBaseI2c.h
 *
 * Base class for I2C-attached display drivers.
 *
 * Holds the I2C bus, address, MUX channel and driver name, and exposes the
 * I2C-specific display API (WriteMessage / WriteValue / Configure*) that the
 * concrete dispDrv* I2C drivers override. Implements the dispDrvBase
 * writeMessage() contract by delegating to WriteMessage(), so the display
 * controller can manage I2C and non-I2C displays through the same pointer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISP_DRV_BASE_I2C_H
#define WS_DISP_DRV_BASE_I2C_H

#include "dispDrvBase.h"
#include <Arduino.h>
#include <Wire.h>
#include <protos/display.pb.h>
#include <protos/i2c.pb.h>

// Shared LED backpack defines (used by dispDrv7Seg and dispDrvQuadAlphaNum)
#ifndef LED_BACKPACK_ALIGNMENT_UNSPECIFIED
#define LED_BACKPACK_ALIGNMENT_UNSPECIFIED                                     \
  0                                    ///< LED backpack alignment not specified
#define LED_BACKPACK_ALIGNMENT_LEFT 1  ///< LED backpack left-aligned text
#define LED_BACKPACK_ALIGNMENT_RIGHT 2 ///< LED backpack right-aligned text
#define LED_BACKPACK_ALIGNMENT_DEFAULT                                         \
  LED_BACKPACK_ALIGNMENT_LEFT     ///< Default LED backpack alignment
#define OLED_STATUS_BAR_HEIGHT 9  ///< OLED status bar: 8px icons + 1px line
#define OLED_STATUS_BAR_ICON_SZ 8 ///< OLED status bar icon size in pixels
#define LED_MAX_CHARS 4 ///< Maximum characters on an LED backpack display
#endif

/*!
    @brief  Base class for I2C-attached display drivers.
*/
class dispDrvBaseI2c : public dispDrvBase {
public:
  /*!
      @brief  Constructor.
      @param  i2c          The I2C hardware interface.
      @param  address      The I2C device's unique address.
      @param  mux_channel  Optional channel number for I2C MUX addressing.
      @param  driver_name  The name of the driver.
      @param  panel        The panel/variant name.
  */
  dispDrvBaseI2c(TwoWire *i2c, uint16_t address, uint32_t mux_channel,
                 const char *driver_name, const char *panel)
      : dispDrvBase(), _i2c(i2c), _address(address),
        _i2c_mux_channel(mux_channel) {
    strncpy(_name, driver_name ? driver_name : "", sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    strncpy(_panel, panel ? panel : "", sizeof(_panel) - 1);
    _panel[sizeof(_panel) - 1] = '\0';
  }

  /*! @brief  Virtual destructor. */
  virtual ~dispDrvBaseI2c() {}

  /*!
      @brief    Gets the name of the driver.
      @returns  The driver's name.
  */
  const char *GetDrvName() { return _name; }

  /*!
      @brief    Gets the name of the panel.
      @returns  The panel's name.
  */
  const char *GetPanelName() { return _panel; }

  /*!
      @brief    Gets the I2C device's address.
      @returns  The I2C device's unique address.
  */
  uint16_t GetAddress() { return _address; }

  /*!
      @brief    Writes a message to the I2C display.
      @param    message  The message to be displayed.
  */
  virtual void WriteMessage(const char *message) {
    // noop
  }

  /*!
      @brief    Writes a floating-point value to the I2C display.
      @param    value  The value to be displayed.
  */
  virtual void WriteValue(float value) {
    // noop
  }

  /*!
      @brief    Writes an integer value to the I2C display.
      @param    value  The value to be displayed.
  */
  virtual void WriteValue(int32_t value) {
    // noop
  }

  /*!
      @brief    Configures a LED backpack.
      @param    brightness  The brightness of the LED backpack.
      @param    alignment   The alignment of the LED backpack.
  */
  virtual void ConfigureI2CBackpack(int32_t brightness, uint32_t alignment) {
    // noop
  }

  /*!
      @brief    Configures a character LCD.
      @param    rows              The number of rows in the LCD.
      @param    cols              The number of columns in the LCD.
      @param    enable_backlight  True to enable the backlight.
  */
  virtual void ConfigureCharLcd(uint32_t rows, uint32_t cols,
                                bool enable_backlight) {
    // noop
  }

  /*!
      @brief    Enables or disables the backlight on a character LCD.
      @param    enable_backlight  True to enable the backlight, False to
                                  disable it.
  */
  virtual void EnableBackLightCharLCD(bool enable_backlight) {
    // noop
  }

  /*!
      @brief    Writes a message to a character LCD via a display Write proto.
      @param    write_msg  Pointer to a ws_display_Write message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool WriteMessageCharLCD(ws_display_Write *write_msg) {
    WriteMessage(write_msg->message);
    return true;
  }

  /*!
      @brief    Sets the brightness of the LED backpack.
      @param    b  Brightness value, from 0 (off) to 15 (full brightness).
  */
  virtual void SetLedBackpackBrightness(uint8_t b) {
    // noop
  }

  /*!
      @brief    Configures an SSD1306-style OLED display. Must be called before
                begin().
      @param    width      The width of the display in pixels.
      @param    height     The height of the display in pixels.
      @param    text_size  The magnification factor for the text size.
  */
  virtual void ConfigureSSD1306(uint8_t width, uint8_t height,
                                uint8_t text_size) {
    // noop
  }

  /*!
      @brief  Satisfies the dispDrvBase contract; routes to WriteMessage().
      @param  message      Null-terminated message to send.

      I2C display drivers expose only a simple message-write API, so cursor
      placement and clear behavior are ignored.
  */
  void writeMessage(const char *message) override { WriteMessage(message); }

protected:
  /*!
      @brief  Draws the compact OLED status bar into the buffer: the
              Adafruit IO username on the left and 8x8 cloud / WiFi /
              battery icons on the right, above a separator line. The caller
              pushes the buffer to the panel.
      @param  gfx          The display's GFX canvas.
      @param  io_username  Adafruit IO username to display.
      @param  fg           Foreground (lit) colour.
      @param  bg           Background colour.
  */
  void drawOledStatusBar(Adafruit_GFX *gfx, const char *io_username,
                         uint16_t fg, uint16_t bg) {
    gfx->fillRect(0, 0, gfx->width(), OLED_STATUS_BAR_HEIGHT, bg);
    gfx->setTextSize(1);
    gfx->setTextColor(fg);
    gfx->setTextWrap(false);
    gfx->setCursor(0, 0);
    // Leave room for the three icons on the right
    int16_t max_chars = (gfx->width() - 3 * (OLED_STATUS_BAR_ICON_SZ + 2)) / 6;
    for (int16_t i = 0; io_username && io_username[i] && i < max_chars; i++)
      gfx->write(io_username[i]);
    gfx->drawFastHLine(0, OLED_STATUS_BAR_HEIGHT - 1, gfx->width(), fg);
    _statusbar_rssi = 0;
    _statusbar_mqtt_connected = true;
    drawOledStatusIcons(gfx, 0, true, fg, bg, true);
  }

  /*!
      @brief  Redraws the OLED status bar icons into the buffer when they
              changed (WiFi by 5 dBm or more), or always when forced.
      @param  gfx          The display's GFX canvas.
      @param  rssi         WiFi RSSI value.
      @param  mqtt_status  True if MQTT is connected.
      @param  fg           Foreground (lit) colour.
      @param  bg           Background colour.
      @param  force        True to redraw even if nothing changed.
      @return True if any icon was redrawn (the caller should refresh).
  */
  bool drawOledStatusIcons(Adafruit_GFX *gfx, int8_t rssi, bool mqtt_status,
                           uint16_t fg, uint16_t bg, bool force = false) {
    bool update_rssi = force || abs(rssi - _statusbar_rssi) >= 5;
    bool update_mqtt = force || mqtt_status != _statusbar_mqtt_connected;
    if (!update_rssi && !update_mqtt)
      return false;
    const int16_t sz = OLED_STATUS_BAR_ICON_SZ;
    const int16_t bat_x = gfx->width() - sz;
    const int16_t wifi_x = bat_x - sz - 2;
    const int16_t cloud_x = wifi_x - sz - 2;
    if (force) {
      gfx->fillRect(bat_x, 0, sz, sz, bg);
      gfx->drawBitmap(bat_x, 0, oled_bmp_bat_full, sz, sz, fg);
    }
    if (update_mqtt) {
      gfx->fillRect(cloud_x, 0, sz, sz, bg);
      gfx->drawBitmap(cloud_x, 0,
                      mqtt_status ? oled_bmp_cloud_online
                                  : oled_bmp_cloud_offline,
                      sz, sz, fg);
      _statusbar_mqtt_connected = mqtt_status;
    }
    if (update_rssi) {
      const unsigned char *wifi_icon = oled_bmp_wifi_no_signal;
      if (rssi >= -50)
        wifi_icon = oled_bmp_wifi_full;
      else if (rssi >= -60)
        wifi_icon = oled_bmp_wifi_fair;
      else if (rssi >= -70)
        wifi_icon = oled_bmp_wifi_weak;
      gfx->fillRect(wifi_x, 0, sz, sz, bg);
      gfx->drawBitmap(wifi_x, 0, wifi_icon, sz, sz, fg);
      _statusbar_rssi = rssi;
    }
    return true;
  }

  TwoWire *_i2c = nullptr;       ///< I2C hardware interface
  uint16_t _address = 0;         ///< I2C device address
  uint32_t _i2c_mux_channel = 0; ///< Optional I2C MUX channel
  char _name[65] = {0};          ///< Driver name
  char _panel[33] = {0};         ///< Panel name
};

#endif // WS_DISP_DRV_BASE_I2C_H
