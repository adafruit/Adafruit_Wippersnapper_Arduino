/*!
 * @file drvOutCharLcd.h
 *
 * Device driver for I2C Character LCDs (HD44780)
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

#ifndef DRV_OUT_CHAR_LCD
#define DRV_OUT_CHAR_LCD

#include "drvOutputBase.h"
#include <Adafruit_LiquidCrystal.h>
#include <Arduino.h>

/*!
    @brief  Class that provides a driver interface for a lcd character display.
            This class is a wrapper around the Adafruit_LiquidCrystal library.
*/
class drvOutCharLcd : public drvOutputBase {
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
  drvOutCharLcd(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name)
      : drvOutputBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvOutPutBase constructor
  }

  /*!
      @brief    Destructor for a quad alphanumeric display.
  */
  ~drvOutCharLcd() {
    if (_lcd) {
      delete _lcd;
      _lcd = nullptr;
    }
  }

  /*!
      @brief    Initializes an I2C character LCD and begins I2C communication.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _lcd = new Adafruit_LiquidCrystal(_address, _i2c);
    bool did_begin = _lcd->begin(_cols, _rows);
    if (did_begin && _enable_backlight) {
      _lcd->setBacklight(HIGH);
    }
    return did_begin;
  }

  /*!
      @brief    Writes a message to the LCD.
      @note     MUST be called prior to begin() to configure the LCD's size
      @param    message
                The message to be displayed.
  */
  void ConfigureCharLcd(uint8_t rows, uint8_t cols, bool enable_backlight) {
    _rows = rows;
    _cols = cols;
    _enable_backlight = enable_backlight;
  }

  /*!
      @brief    Writes a message to the LCD.
      @param    message
                The message to be displayed.
  */
  void WriteMessageCharLCD(const char *message) override {
    if (_lcd == nullptr)
      return;

    // Before writing, let's clear the display
    _lcd->clear();

    size_t message_length = strlen(message);
    size_t cur_idx = 0; // Current index in the message

    // Write each row until it hits: \n, or the end of the message, or the last
    // column/row position
    for (int cur_row = 0; cur_row < _rows && cur_idx < message_length;
         cur_row++) {
      // Write each row out at the beginning of the row
      _lcd->setCursor(0, cur_row);
      for (int cur_col = 0; cur_col < _cols && cur_idx < message_length;
           cur_col++) {
        char c = message[cur_idx];
        if (c == '\n') {
          cur_idx++;
          break;
        }
        _lcd->write(c);
        cur_idx++;
      }
    }
  }

protected:
  Adafruit_LiquidCrystal *_lcd =
      nullptr;            ///< Pointer to the Adafruit_LiquidCrystal object
  uint8_t _rows;          ///< Number of rows in the display
  uint8_t _cols;          ///< Number of columns in the display
  bool _enable_backlight; ///< Flag to enable/disable backlight
};

#endif // DRV_OUT_CHAR_LCD