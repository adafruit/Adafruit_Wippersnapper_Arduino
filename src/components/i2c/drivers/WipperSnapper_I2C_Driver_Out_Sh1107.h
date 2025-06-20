/*!
 * @file WipperSnapper_I2C_Driver_Out_SH1107.h
 *
 *  Device driver for a SH1107 OLED Display
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

#ifndef WIPPERSNAPPER_I2C_DRIVER_OUT_SH1107_H
#define WIPPERSNAPPER_I2C_DRIVER_OUT_SH1107_H

#include "WipperSnapper_I2C_Driver_Out.h"
// #include <Adafruit_GrayOLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>

#define DEFAULT_WIDTH 128 ///< Default width for a sh1107 128x64 display
#define DEFAULT_HEIGHT 64 ///< Default height for a sh1107 128x64 display

/*!
    @brief  Class that provides a driver interface for a SH1107
   OLED Display
*/
class WipperSnapper_I2C_Driver_Out_SH1107
    : public WipperSnapper_I2C_Driver_Out {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a  OLED display.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_Out_SH1107(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver_Out(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _width = DEFAULT_WIDTH;
    _height = DEFAULT_HEIGHT;
  }

  /*!
      @brief    Destructor for a SH1107 OLED display.
  */
  ~WipperSnapper_I2C_Driver_Out_SH1107() {
    if (_display != nullptr) {
      _display->clearDisplay();
      _display->display();
      _display->oled_command(SH110X_DISPLAYOFF);
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief    Initializes the SH1107 display and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() {
    // Attempt to create and allocate a SH1107 obj, backwards w/h
    _display = new Adafruit_SH1107(_height, _width, _i2c);
    if (!_display->begin(_sensorAddress, true))
      return false;

    // Show image buffer on the display hardware.
    // Since the buffer is intialized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    _display->display();
    delay(1700);

    // Clear the buffer.
    _display->clearDisplay();
    _display->display();
    _display->setRotation(1);

    // Configure the text size and color
    _display->setTextSize(_text_sz);
    _display->setTextColor(SH110X_WHITE);
    _display->setCursor(0, 0);
    // Use full 256 char 'Code Page 437' font
    // _display->cp437(true);
    // Clear the buffer
    _display->clearDisplay();
    _display->display();
    _display->print(char('a'));
    delay(500);
    _display->write(char('b'));
    _display->display();
    return true;
  }

  /*!
      @brief    Configures a SH1107 OLED display. Must be called before driver
     begin()
      @param    width
                  The width of the display in pixels.
      @param    height
                  The height of the display in pixels.
      @param    text_size
                  The magnification factor for the text size.
  */
  void ConfigureSH1107(uint8_t width, uint8_t height, uint8_t text_size) {
    _width = width;
    _height = height;
    _text_sz = text_size;
    WS_DEBUG_PRINT("SH1107 text size: ");
    WS_DEBUG_PRINTLN(text_size);
  }
  /*!
      @brief    Configures a SSD1306 OLED display. Must be called before driver
     begin() - This is a fake function to match the SSD1306 interface.
      @param    width
                  The width of the display in pixels.
      @param    height
                  The height of the display in pixels.
      @param    text_size
                  The magnification factor for the text size.
  */
  void ConfigureSSD1306(uint8_t width, uint8_t height, uint8_t text_size) {
    // This is a SH1107, not a SSD1306, so we don't need to do anything here.
    ConfigureSH1107(width, height, text_size);
  }

  /*!
      @brief    Writes a message to the SH1107 display.
      @param    message
                  The message to be displayed.
  */
  void WriteMessageSH1107(const char *message) {
    if (_display == nullptr)
      return;
    WS_DEBUG_PRINT("SH1107 Message:");
    WS_DEBUG_PRINTLN(message);
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
          WS_DEBUG_PRINT("SH1107 Newline at: ");
          WS_DEBUG_PRINTLN(y_idx);
          _display->setCursor(0, y_idx);
          i += 3;
        } else if (message[i + 1] == 'n') {
          // Skip to the next line
          y_idx += line_height;
          WS_DEBUG_PRINT("SH1107 Newline at: ");
          WS_DEBUG_PRINTLN(y_idx);
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

  /*!
      @brief    Writes a message to the fake "SSD1306" SH1107 display.
      @param    message
                  The message to be displayed.
  */
  void WriteMessageSSD1306(const char *message) {
    // This is a SH1107, not a SSD1306, so we just call the SH1107 write
    WriteMessageSH1107(message);
  }

protected:
  Adafruit_SH1107 *_display =
      nullptr;      ///< Pointer to the Adafruit_SH1107 object
  uint8_t _width;   ///< Width of the display in pixels
  uint8_t _height;  ///< Height of the display in pixels
  uint8_t _text_sz; ///< Text size of the display
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_SH1107_H
