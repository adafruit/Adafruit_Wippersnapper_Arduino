/*!
 * @file src/components/display/drivers/dispDrvCharLcdPcf8574.h
 *
 * Device driver for HD44780 Character LCDs on a generic PCF8574 I2C backpack
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry for Adafruit Industries 2026
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WS_DISP_DRV_CHAR_LCD_PCF8574_H
#define WS_DISP_DRV_CHAR_LCD_PCF8574_H

#include "dispDrvBaseI2c.h"
#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>

/*!
    @brief  Class that provides a driver interface for a lcd character display
            on a generic PCF8574 I2C backpack. This class is a wrapper around
            the LiquidCrystal_PCF8574 library, which takes the I2C bus as a
            TwoWire (unlike LiquidCrystal_I2C / hd44780, which use Wire).
*/
class dispDrvCharLcdPcf8574 : public dispDrvBaseI2c {
public:
  /*!
      @brief    Constructor for a PCF8574 lcd character display.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
      @param    panel
                The panel/variant name.
  */
  dispDrvCharLcdPcf8574(TwoWire *i2c, uint16_t sensorAddress,
                        uint32_t mux_channel, const char *driver_name,
                        const char *panel)
      : dispDrvBaseI2c(i2c, sensorAddress, mux_channel, driver_name, panel) {}

  /*!
      @brief    Destructor.
  */
  ~dispDrvCharLcdPcf8574() {
    if (_lcd) {
      delete _lcd;
      _lcd = nullptr;
    }
  }

  /*!
      @brief    Initializes a PCF8574 I2C character LCD and begins I2C
                communication.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    // LiquidCrystal_PCF8574::begin() returns void, so probe the backpack
    // first to report a missing device.
    _i2c->beginTransmission((uint8_t)_address);
    if (_i2c->endTransmission() != 0)
      return false;
    // Default constructor = the common backpack wiring (P0=RS, P1=RW, P2=EN,
    // P3=backlight, P4-P7=D4-D7).
    // TODO(verify on HIL): confirm on a generic PCF8574 16x2 backpack.
    _lcd = new LiquidCrystal_PCF8574((uint8_t)_address);
    if (!_lcd)
      return false;
    _lcd->begin(_cols, _rows, *_i2c);
    _lcd->setBacklight(_enable_backlight ? 255 : 0);
    _lcd->clear();
    return true;
  }

  /*!
      @brief    Configures the LCD's size and backlight.
      @note     MUST be called prior to begin() to configure the LCD's size
      @param   rows
                The number of rows in the LCD.
      @param   cols
              The number of columns in the LCD.
      @param   enable_backlight
              True if the backlight is enabled, False otherwise.
  */
  void ConfigureCharLcd(uint32_t rows, uint32_t cols,
                        bool enable_backlight) override {
    _rows = rows;
    _cols = cols;
    _enable_backlight = enable_backlight;
  }

  /*!
       @brief    Enables or disables the backlight on a character LCD.
       @param    enable_backlight
                   True to enable the backlight, False to disable it.
 */
  void EnableBackLightCharLCD(bool enable_backlight) override {
    if (_lcd == nullptr)
      return;
    _lcd->setBacklight(enable_backlight ? 255 : 0);
    _enable_backlight = enable_backlight;
  }

  /*!
      @brief    Writes a message to the LCD.
      @param    message
                The message to be displayed.
  */
  void WriteMessage(const char *message) override {
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
        char parsed_char = 0;
        bool is_newline = false;
        if (parseWriteToken(message, message_length, cur_idx, parsed_char,
                            is_newline, char(0xDF))) {
          if (is_newline) {
            cur_idx++;
            break;
          }
          _lcd->write(parsed_char);
          cur_idx++;
          continue;
        }

        _lcd->write(message[cur_idx]);
        cur_idx++;
      }
    }
  }

protected:
  LiquidCrystal_PCF8574 *_lcd =
      nullptr;        ///< Pointer to the LiquidCrystal_PCF8574 object
  uint8_t _rows = 2;  ///< Number of rows in the display
  uint8_t _cols = 16; ///< Number of columns in the display
  bool _enable_backlight = true; ///< Flag to enable/disable backlight
};

#endif // WS_DISP_DRV_CHAR_LCD_PCF8574_H
