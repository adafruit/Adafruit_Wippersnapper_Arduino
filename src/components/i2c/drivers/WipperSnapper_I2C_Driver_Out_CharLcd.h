/*!
 * @file WipperSnapper_I2C_Driver_Out_CharLcd.h
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

#ifndef WIPPERSNAPPER_I2C_DRIVER_OUT_CHARLCD_H
#define WIPPERSNAPPER_I2C_DRIVER_OUT_CHARLCD_H

#include "WipperSnapper_I2C_Driver_Out.h"
#include <Adafruit_LiquidCrystal.h>

/*!
    @brief  Class that provides a driver interface for a lcd character display.
            This class is a wrapper around the Adafruit_LiquidCrystal library.
*/
class WipperSnapper_I2C_Driver_Out_CharLcd
    : public WipperSnapper_I2C_Driver_Out {

public:
  /*!
      @brief    Constructor for a LCD character display.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  WipperSnapper_I2C_Driver_Out_CharLcd(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver_Out(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*!
      @brief    Destructor for an MS8607 sensor.
  */
  ~WipperSnapper_I2C_Driver_Out_CharLcd() {
    if (_lcd != nullptr) {
      delete _lcd;
      _lcd = nullptr;
    }
  }

  /*!
      @brief    Initializes the drvOutQuadAlphaNum component and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() {
    _lcd = new Adafruit_LiquidCrystal(_sensorAddress, _i2c);
    bool did_begin = _lcd->begin(_cols, _rows);
    if (did_begin) {
      _lcd->setBacklight(HIGH);
    }
    return did_begin;
  }

  /*!
      @brief    Writes a message to the LCD.
      @note     MUST be called prior to begin() to configure the LCD's size
      @param   rows
                The number of rows in the LCD.
      @param   cols
              The number of columns in the LCD.
  */
  void ConfigureCharLcd(uint8_t rows, uint8_t cols) {
    _rows = rows;
    _cols = cols;
  }

  /*!
    @brief    Turns the character LCD backlight on or off.
    @param    enable
                True to enable the backlight, false to disable it.
  */
  void EnableCharLcdBacklight(bool enable = true) {
    if (_lcd == nullptr)
      return;

    if (enable) {
      _lcd->setBacklight(HIGH);
    } else {
      _lcd->setBacklight(LOW);
    }
  }

  /*!
      @brief    Writes a message to the LCD.
      @param    message
                The message to be displayed.
  */
  void WriteMessage(const char *message) {
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
        if (c == '\\' && cur_idx + 1 < message_length &&
            message[cur_idx + 1] == 'n') {
          cur_idx += 2; // Skip the '\n' character in the buffer
          break;        // and move to the next row
        } else if (c == '\\' && cur_idx + 1 < message_length &&
                   message[cur_idx + 1] == 'r') {
          cur_idx += 2; // Skip the '\r' character in the buffer
          continue;     // and continue writing on the same row
        } else if (c == 194 && cur_idx + 1 < message_length &&
                   message[cur_idx + 1] == 176) {
          cur_idx += 2;      // Skip the degree symbol sequence in the buffer
          _lcd->write(0xDF); // and write the degree symbol
        } else {
          _lcd->write(c);
          cur_idx++;
        }
      }
    }
  }

protected:
  Adafruit_LiquidCrystal *_lcd =
      nullptr;   ///< Pointer to the Adafruit_LiquidCrystal object
  uint8_t _rows; ///< Number of rows in the display
  uint8_t _cols; ///< Number of columns in the display
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_CHARLCD_H