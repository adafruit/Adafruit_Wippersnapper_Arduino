/*!
 * @file drvOutSsd1306.h
 *
 * Device driver for OLED displays with a SSD1306 driver
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

#ifndef DRV_OUT_SSD1306_H
#define DRV_OUT_SSD1306_H

#include "drvOutputBase.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

/*!
    @brief  Class that provides a driver interface for a SSD1306 OLED display.
            This class is a wrapper around the Adafruit_SSD1306 library.
*/
class drvOutSsd1306 : public drvOutputBase {
public:
  /*!
      @brief    Constructor for a lcd character display.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvOutSsd1306(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name)
      : drvOutputBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvOutPutBase constructor
  }

  /*!
      @brief    Destructor for a quad alphanumeric display.
  */
  ~drvOutSsd1306() {
    if (_display != nullptr) {
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
    if (!_display->begin(SSD1306_SWITCHCAPVCC, _address))
      return false;
    // Clear the buffer
    _display->clearDisplay();
    // Configure the text size and color
    _display->setTextSize(_text_sz);
    _display->setTextColor(SSD1306_WHITE);
    // Reset the cursor position
    _display->setCursor(0, 0);
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

#endif // DRV_OUT_SSD1306_H