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
#define DEFAULT_ADDR 0x3C
#define PIN_OLED_RESET -1

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
  WipperSnapper_I2C_Driver_Out_Ssd1306(TwoWire *i2c,
                                       uint16_t sensorAddress)
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
    _display = new Adafruit_SSD1306(_width, _height, &_i2c, PIN_OLED_RESET);
    bool did_begin = _display->begin(SSD1306_SWITCHCAPVCC, DEFAULT_ADDR);
    if (! did_begin)
      return false;

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    _display->display();
    delay(2000);
    // Clear the buffer
    _display->clearDisplay();
    // Set the text size

    return true;
  }


protected:
  Adafruit_SSD1306 *_display = nullptr; ///< Pointer to the Adafruit_SSD1306 object
  uint8_t _width, _height, _text_sz; ///< Width and height of the display
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_SSD1306_H