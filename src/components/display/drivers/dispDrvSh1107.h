/*!
 * @file src/components/display/drivers/dispDrvSh1107.h
 *
 * Device driver for OLED displays with a SH1107 driver
 * (e.g., 128x64 OLED FeatherWing)
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry for Adafruit Industries 2025
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WS_DISP_DRV_SH1107_H
#define WS_DISP_DRV_SH1107_H

#include "dispDrvBaseI2c.h"
#include <Adafruit_SH110X.h>
#include <Arduino.h>

#define OLED_128X64_WING_WIDTH 128 ///< 128x64 OLED FeatherWing width in pixels
#define OLED_128X64_WING_HEIGHT 64 ///< 128x64 OLED FeatherWing height in pixels
#define OLED_128X64_WING_ROTATION_90                                           \
  1 ///< Rotation value for 128x64 FeatherWing landscape orientation

/*!
    @brief  Class that provides a driver interface for a SH1107 OLED display.
*/
class dispDrvSh1107 : public dispDrvBaseI2c {
public:
  /*!
      @brief  Constructor for the SH1107 OLED display driver.
      @param  i2c            The I2C hardware interface.
      @param  sensorAddress  The I2C address of the display.
      @param  mux_channel    Optional I2C MUX channel for the display.
      @param  driver_name    The name of the driver.
  */
  dispDrvSh1107(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name, const char *panel)
      : dispDrvBaseI2c(i2c, sensorAddress, mux_channel, driver_name, panel) {}

  ~dispDrvSh1107() {
    if (_display != nullptr) {
      _display->clearDisplay();
      _display->display();
      _display->oled_command(SH110X_DISPLAYOFF);
      delete _display;
      _display = nullptr;
    }
  }

  bool begin() override {
    if ((_width == OLED_128X64_WING_WIDTH &&
         _height == OLED_128X64_WING_HEIGHT &&
         _rotation == OLED_128X64_WING_ROTATION_90) ||
        strcasecmp(_panel, "128x64featherwing") == 0) {
      // FeatherWing needs swapped w/h ctor args
      _display = new Adafruit_SH1107(_height, _width, _i2c);
    } else {
      _display = new Adafruit_SH1107(_width, _height, _i2c);
    }
    if (!_display->begin(_address, true))
      return false;

    _display->clearDisplay();
    _display->display();
    _display->setRotation(_rotation);
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    _display->setCursor(0, 0);
    _display->clearDisplay();
    _display->display();
    return true;
  }

  void ConfigureSSD1306(uint8_t width, uint8_t height,
                        uint8_t text_size) override {
    _width = width;
    _height = height;
    _text_sz = text_size;
    // SH1107 requires rotation 0-3, not degrees
    _rotation =
        (width == OLED_128X64_WING_WIDTH && height == OLED_128X64_WING_HEIGHT)
            ? OLED_128X64_WING_ROTATION_90
            : 0;
  }

  void WriteMessage(const char *message) override {
    if (_display == nullptr)
      return;

    int16_t y_idx = 0;
    _display->clearDisplay();
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    _display->setCursor(0, y_idx);
    _display->display();

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
          _display->display();
        }
        continue;
      }

      _display->print(message[i]);
      _display->display();
    }
  }

protected:
  Adafruit_SH1107 *_display = nullptr; ///< SH1107 driver instance
};

#endif // WS_DISP_DRV_SH1107_H
