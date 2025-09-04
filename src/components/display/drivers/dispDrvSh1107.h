/*!
 * @file src/components/display/drivers/dispDrvSh1107.h
 *
 *  Device driver for a SH1107 OLED Display
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

#include "dispDrvBase.h"
#include <Adafruit_SH110X.h>

#define WS_SH1107_DEFAULT_WIDTH                                                \
  128 ///< Default width for a sh1107 128x64 display
#define WS_SH1107_DEFAULT_HEIGHT                                               \
  64 ///< Default height for a sh1107 128x64 display

#define OLED_128X64_WING_WIDTH 128 ///< Width of the 128x64 OLED FeatherWing
#define OLED_128X64_WING_HEIGHT 64 ///< Height of the 128x64 OLED FeatherWing
#define OLED_128X64_WING_ROTATION_90 1 ///< Rotation of OLED FeatherWing 0-3

/*!
    @brief  Class that provides a driver interface for a SH1107
   OLED Display
*/
class dispDrvSh1107 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the SH1107 display driver.
  */
  dispDrvSh1107(TwoWire *i2c, uint16_t sensorAddress)
      : dispDrvBase(i2c, sensorAddress), _display(nullptr) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _width = WS_SH1107_DEFAULT_WIDTH;
    _height = WS_SH1107_DEFAULT_HEIGHT;
  }

  /*!
      @brief  Destructor for a SH1107 display driver.
  */
  ~dispDrvSh1107() {
    if (_display != nullptr) {
      _display->clearDisplay();
      _display->display();
      _display->oled_command(SH110X_DISPLAYOFF);
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the SSD1306 display driver.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin() override {
    if (!_i2c)
      return false;

    // Attempt to create and allocate a SH1107 obj.
    if (_width == OLED_128X64_WING_WIDTH &&
        _height == OLED_128X64_WING_HEIGHT &&
        _rotation == OLED_128X64_WING_ROTATION_90) {
      // FeatherWing needs to be rotated 90deg and swap w/h ctor args
      _display = new Adafruit_SH1107(_height, _width, _i2c);
    } else {
      _display = new Adafruit_SH1107(_width, _height, _i2c);
    }
    if (!_display->begin(_sensorAddress, true))
      return false;

    // Clear the buffer.
    _display->clearDisplay();
    _display->display();
    _display->setRotation(_rotation); // 0-3, not degrees for SH1107

    // Configure the text size and color
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    _display->setCursor(0, 0);
    // Clear the buffer
    _display->clearDisplay();
    _display->display();
    return true;
  }

  /*!
      @brief  Sets the text size for the display.
      @param  s
              The text size to set.
      @note   This method overrides the base class method to provide specific
              functionality for the SSD1306 driver.
  */
  void setTextSize(uint8_t s) override {
    if (!_display)
      return;
    _text_sz = s;
    _display->setTextSize(s);
  }

  void setRotation(uint8_t r) override {
    if (!_display)
      return;
    _rotation = r % 90; // constrain to 0-3
    _display->setRotation(r);
  }

  /*!
      @brief  Writes a message to the display.
      @param  message
              The message to write to the display.
      @note   This method overrides the base class method to provide specific
              functionality for the SSD1306 driver.
  */
  virtual void writeMessage(const char *message) override {
    if (_display == nullptr)
      return;
    // Start with a fresh display buffer
    // and settings
    int16_t y_idx = 0;
    _display->clearDisplay();
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    _display->setCursor(0, y_idx);
    _display->display();

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);
    for (size_t i = 0; i < msg_size && c_idx < msg_size; i++) {
      if (message[i] == '\\' && i + 1 < msg_size &&
          (message[i + 1] == 'n' || message[i + 1] == 'r')) {
        // Handle \r\n sequence as a single newline
        if (message[i + 1] == 'r' && i + 3 < msg_size &&
            message[i + 2] == '\\' && message[i + 3] == 'n') {
          // Skip to the next line
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i += 3;
        } else if (message[i + 1] == 'n') {
          // Skip to the next line
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i++;
        }
      } else if (message[i] == 0xC2 && message[i + 1] == 0xB0) {
        _display->write(char(248));
        _display->display();
        i++;
      } else {
        _display->print(message[i]);
        _display->display();
      }
    }
  }

private:
  Adafruit_SH1107 *_display;
};

#endif // WS_DISP_DRV_SSD1306