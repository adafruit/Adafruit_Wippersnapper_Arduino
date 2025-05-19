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

#define DEFAULT_WIDTH 128
#define DEFAULT_HEIGHT 64

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
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief    Initializes the SSD1306 display and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() {
    _display = new Adafruit_SSD1306(_width, _height, _i2c);
    bool did_begin = _display->begin(
        SSD1306_SWITCHCAPVCC,
        0x3C); // TODO: Note that this is hardcoded, not sure why not init'd yet
    if (!did_begin)
      return false;

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    _display->display();
    delay(1000);
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
      @param    i2c_address
                  The I2C address of the display.
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
    _display->clearDisplay();

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    WS_DEBUG_PRINT("Line height: ");
    WS_DEBUG_PRINTLN(line_height);

    int16_t y_idx = 0;
    _display->setCursor(0, y_idx);
    for (int i = 0; message[i] != '\0'; i++) {
      if (message[i] == '\n') {
        WS_DEBUG_PRINTLN("New line detected!");
        y_idx += line_height;
        _display->setCursor(0, y_idx);
      } else {
        WS_DEBUG_PRINT("Printing char: ");
        WS_DEBUG_PRINT(message[i]);
        WS_DEBUG_PRINT(" at y: ");
        WS_DEBUG_PRINTLN(y_idx);
        _display->print(message[i]);
      }
    }
    _display->display();
  }

protected:
  Adafruit_SSD1306 *_display =
      nullptr;                       ///< Pointer to the Adafruit_SSD1306 object
  uint8_t _width, _height, _text_sz; ///< Width and height of the display
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_SSD1306_H