/*!
 * @file WipperSnapper_I2C_Driver_Out_Ssd1306.h
 *
 *  Device driver for SSD1306 Monochrome I2C OLED displays
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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/*!
    @brief  Class that provides a driver interface for SSD1306 OLED displays
*/
class WipperSnapper_I2C_Driver_Out_Ssd1306
    : public WipperSnapper_I2C_Driver_Out {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an SSD1306 display.
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
  }

  /*!
      @brief    Destructor for an SSD1306 display.
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
    bool did_begin = _display->begin(SSD1306_SWITCHCAPVCC, _sensorAddress);
    return did_begin;
  }

  /*!
      @brief    Writes a message to an SSD1306 display.
      @param    message
                  The message to be displayed.
  */
  void WriteMessage(const char *message) {

  }

protected:
  Adafruit_SSD1306 *_display = 
      nullptr;         ///< ptr to an SSD1306 display object
  int32_t _width;      ///< Width of the display in pixels
  int32_t _height;     ///< Height of the display in pixels
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_SSD1306_H