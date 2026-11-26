/*!
 * @file src/components/display/drivers/dispDrvSh1106.h
 *
 * Device driver for OLED displays with a SH1106 driver
 * (e.g., generic 1.3" 128x64 OLED)
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry for Adafruit Industries 2026
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WS_DISP_DRV_SH1106_H
#define WS_DISP_DRV_SH1106_H

#include "dispDrvBaseI2c.h"
#include <Adafruit_SH110X.h>
#include <Arduino.h>

/*!
    @brief  Class that provides a driver interface for a SH1106 OLED display.
            This class is a wrapper around the Adafruit_SH1106G class.
*/
class dispDrvSh1106 : public dispDrvBaseI2c {
public:
  /*!
      @brief    Constructor for a SH1106 OLED display driver.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
      @param    panel
                The panel/variant name.
  */
  dispDrvSh1106(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name, const char *panel)
      : dispDrvBaseI2c(i2c, sensorAddress, mux_channel, driver_name, panel) {}

  /*!
      @brief    Destructor.
  */
  ~dispDrvSh1106() {
    if (_display != nullptr) {
      _display->oled_command(SH110X_DISPLAYOFF);
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief    Initializes the SH1106 display and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _display = new Adafruit_SH1106G(_width, _height, _i2c);
    if (!_display || !_display->begin(_address, true))
      return false;
    // Clear the buffer
    _display->clearDisplay();
    // Configure the text size and color
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    // Reset the cursor position
    _display->setCursor(0, 0);
    _display->display();
    return true;
  }

  /*!
      @brief    Configures a SH1106 OLED display. Must be called before driver
     begin()
      @param    width
                  The width of the display in pixels.
      @param    height
                  The height of the display in pixels.
      @param    text_size
                  The magnification factor for the text size.
  */
  void ConfigureSSD1306(uint8_t width, uint8_t height,
                        uint8_t text_size) override {
    _width = width;
    _height = height;
    _text_sz = text_size;
  }

  /*!
      @brief    Writes a message to the SH1106 display.
      @param    message
                  The message to be displayed.
  */
  /*!
      @brief  Draws the compact status bar (username and icons).
      @param  io_username  Adafruit IO username to display.
  */
  void drawStatusBar(const char *io_username) override {
    if (_display == nullptr)
      return;
    _display->clearDisplay();
    drawOledStatusBar(_display, io_username, SH110X_WHITE, SH110X_BLACK);
    _display->display();
  }

  /*!
      @brief  Updates the status bar icons when the state changed.
      @param  rssi         Current WiFi RSSI value.
      @param  bat          Current battery level (0-100, unused).
      @param  mqtt_status  True if MQTT is connected.
  */
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    (void)bat;
    if (_display == nullptr)
      return;
    if (drawOledStatusIcons(_display, rssi, mqtt_status, SH110X_WHITE,
                            SH110X_BLACK))
      _display->display();
  }

  void WriteMessage(const char *message) override {
    if (_display == nullptr)
      return;

    // Start with a fresh display buffer
    // and settings
    // Keep the status bar strip when enabled; otherwise use the whole panel
    int16_t y_idx = _status_bar ? OLED_STATUS_BAR_HEIGHT + 1 : 0;
    if (_status_bar)
      _display->fillRect(0, OLED_STATUS_BAR_HEIGHT, _display->width(),
                         _display->height() - OLED_STATUS_BAR_HEIGHT,
                         SH110X_BLACK);
    else
      _display->clearDisplay();
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    _display->setCursor(0, y_idx);

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    size_t msg_size = strlen(message);
    for (size_t i = 0; i < msg_size; i++) {
      char parsed_char = 0;
      bool is_newline = false;
      if (parseWriteToken(message, msg_size, i, parsed_char, is_newline,
                          char(247))) {
        if (is_newline) {
          y_idx += line_height;
          _display->setCursor(0, y_idx);
        } else {
          _display->write(parsed_char);
        }
        continue;
      }

      _display->print(message[i]);
    }
    // Push the frame once per message, not per character (I2C bandwidth)
    _display->display();
  }

protected:
  Adafruit_SH1106G *_display =
      nullptr; ///< Pointer to the Adafruit_SH1106G object
};

#endif // WS_DISP_DRV_SH1106_H
