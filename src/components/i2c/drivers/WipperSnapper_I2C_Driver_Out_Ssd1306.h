/*!
 * @file WipperSnapper_I2C_Driver_Out_Ssd1306.h
 *
 *  Device driver for a SSD1306 OLED Display
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell for Adafruit Industries 2025
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_I2C_DRIVER_OUT_SSD1306_H
#define WIPPERSNAPPER_I2C_DRIVER_OUT_SSD1306_H

#include "WipperSnapper_I2C_Driver_Out.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define DEFAULT_WIDTH 128 ///< Default width for a ssd1306 128x64 display
#define DEFAULT_HEIGHT 64 ///< Default height for a ssd1306 128x64 display

/*!
    @brief  Class that provides a driver interface for a SSD1306
   OLED Display
*/
class WipperSnapper_I2C_Driver_Out_Ssd1306
    : public WipperSnapper_I2C_Driver_Out {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SSD1306 OLED display.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_Out_Ssd1306(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver_Out(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _width = DEFAULT_WIDTH;
    _height = DEFAULT_HEIGHT;
  }

  /*!
      @brief    Destructor for a SSD1306 OLED display.
  */
  ~WipperSnapper_I2C_Driver_Out_Ssd1306() {
    if (_display != nullptr) {
      _display->clearDisplay();
      _display->display();
      _display->ssd1306_command(SSD1306_DISPLAYOFF);
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief    Initializes the SSD1306 display and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() {
    // Attempt to create and allocate a SSD1306 obj.
    _display = new Adafruit_SSD1306(_width, _height, _i2c);
    if (!_display->begin(SSD1306_SWITCHCAPVCC, _sensorAddress))
      return false;
    // Configure the text size and color
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
      @brief    Configures a SSD1306 OLED display. Must be called before driver
     begin()
      @param    width
                  The width of the display in pixels.
      @param    height
                  The height of the display in pixels.
      @param    text_size
                  The magnification factor for the text size.
  */
  void ConfigureSSD1306(uint8_t width, uint8_t height, uint8_t text_size) {
    _width = width;
    _height = height;
    _text_sz = text_size;
  }

  /*!
      @brief    Writes a message to the SSD1306 display.
      @param    message
                  The message to be displayed.
  */
  void WriteMessageSSD1306(const char *message) {
    if (_display == nullptr)
      return;

    // Start with a fresh display buffer
    // and settings
    int16_t y_idx = 0;
    _display->clearDisplay();
    _display->setTextSize(_text_sz);
    _display->setTextColor(SSD1306_WHITE);
    _display->setCursor(0, y_idx);
    _display->display();

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);
    for (size_t i = 0; i < msg_size && c_idx < msg_size; i++) {
      if (message[i] == '\\' && i + 1 < msg_size && message[i + 1] == 'n') {
        // detected a newline char sequence (\n)
        i++;
        // Skip to the next possible line
        y_idx += line_height;
        _display->setCursor(0, y_idx);
      } else if (message[i] == '\\' && i + 1 < msg_size &&
                 message[i + 1] == 'r') {
        // skip the \r character, continue to the next character
        i++;
        continue;
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

protected:
  Adafruit_SSD1306 *_display =
      nullptr;      ///< Pointer to the Adafruit_SSD1306 object
  uint8_t _width;   ///< Width of the display in pixels
  uint8_t _height;  ///< Height of the display in pixels
  uint8_t _text_sz; ///< Text size of the display
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_SSD1306_H