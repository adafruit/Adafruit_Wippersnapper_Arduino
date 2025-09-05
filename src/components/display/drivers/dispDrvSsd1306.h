/*!
 * @file src/components/display/drivers/dispDrvSsd1306.h
 *
 * Driver for SSD1306-based OLED displays.
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
#ifndef WS_DISP_DRV_SSD1306
#define WS_DISP_DRV_SSD1306

#include "dispDrvBase.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define WS_SSD1306_DEFAULT_WIDTH                                               \
  128 ///< Default width for a ssd1306 128x64 display
#define WS_SSD1306_DEFAULT_HEIGHT                                              \
  64 ///< Default height for a ssd1306 128x64 display

/*!
    @brief  Driver for SSD1306-based TFT displays.
*/
class dispDrvSsd1306 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the SSD1306 display driver.
        @param  i2c
                The I2C hardware interface, default is Wire.
        @param  sensorAddress
                The I2C sensor's unique address.
  */
  dispDrvSsd1306(TwoWire *i2c, uint16_t sensorAddress)
      : dispDrvBase(i2c, sensorAddress), _display(nullptr) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _width = WS_SSD1306_DEFAULT_WIDTH;
    _height = WS_SSD1306_DEFAULT_HEIGHT;
  }

  ~dispDrvSsd1306() {
    if (_display) {
      _display->clearDisplay();
      _display->display();
      _display->ssd1306_command(SSD1306_DISPLAYOFF);
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the SSD1306 display driver.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin() override {
    if (_i2c == nullptr)
      return false;
    // Attempt to create and allocate a SSD1306 obj.
    _display = new Adafruit_SSD1306(_width, _height, _i2c);
    if (!_display->begin(SSD1306_SWITCHCAPVCC, _sensorAddress))
      return false;
    // Configure the rotation, text size and color
    _display->setRotation(_rotation);
    _display->setTextSize(_text_sz);
    _display->setTextColor(SSD1306_WHITE);
    // Use full 256 char 'Code Page 437' font
    _display->cp437(true);
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
  Adafruit_SSD1306 *_display;
};

#endif // WS_DISP_DRV_SSD1306