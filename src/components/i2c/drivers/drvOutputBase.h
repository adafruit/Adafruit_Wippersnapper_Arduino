/*!
 * @file drvOutputBase.h
 *
 * Base implementation for I2C output device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_OUTPUT_BASE_H
#define DRV_OUTPUT_BASE_H
#include "drvBase.h"
#include <protos/i2c.pb.h>
#include <protos/i2c_output.pb.h>

/*!
    @brief  Base class for I2C Output Drivers.
*/
class drvOutputBase : public drvBase {

public:
  /*!
      @brief    Instantiates an I2C output device.
      @param    i2c
                The I2C hardware interface, default is Wire.
      @param    address
                The I2C device's unique address.
      @param    mux_channel
                An optional channel number used to address a device on a I2C
     MUX.
      @param    driver_name
                The name of the driver.
  */
  drvOutputBase(TwoWire *i2c, uint16_t address, uint32_t mux_channel,
                const char *driver_name)
      : drvBase(i2c, address, mux_channel, driver_name) {
    // TODO
  }

  /*!
      @brief    Destructor for an I2C output device.
  */
  virtual ~drvOutputBase() {}

  /*!
      @brief    Initializes the I2C output device and begins I2C.
      @param    message
                The message to be displayed.
  */
  virtual void WriteMessage(const char *message) {
    // noop
  }

  /*!
      @brief    Writes a floating point value to the quad alphanumeric display.
      @param    value
                  The value to be displayed. Only the first four digits are
     displayed.
  */
  virtual void WriteValue(float value) {
    // noop
  }

  /*!
      @brief    Writes a floating point value to the quad alphanumeric display.
      @param    value
                  The value to be displayed. Only the first four digits are
     displayed.
  */
  virtual void WriteValue(int32_t value) {
    // noop
  }

  /*!
      @brief    Configures a LED backpack.
      @param    brightness
                The brightness of the LED backpack.
      @param    alignment
                The alignment of the LED backpack.
  */
  virtual void ConfigureI2CBackpack(int32_t brightness, uint32_t alignment) {
    // noop
  }

  /*!
      @brief    Configures a character LCD.
      @param    rows
                  The number of rows in the LCD.
      @param    cols
                  The number of columns in the LCD.
      @param    enable_backlight
                  True if the backlight is enabled, False otherwise.
  */
  virtual void ConfigureCharLcd(uint32_t rows, uint32_t cols,
                                bool enable_backlight) {
    // noop
  }

  /*!
       @brief    Enables or disables the backlight on a character LCD.
       @param    enable_backlight
                   True to enable the backlight, False to disable it.
 */
  void EnableBackLightCharLCD(bool enable_backlight) {
    // noop
  }

  /*!
      @brief    Writes a message to the LCD.
      @param    write_char_lcd
                Pointer to a ws_i2c_output_CharLCDWrite message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool
  WriteMessageCharLCD(ws_i2c_output_CharLCDWrite *write_char_lcd) {
    EnableBackLightCharLCD(write_char_lcd->enable_backlight);
    WriteMessage(write_char_lcd->message);
    return true;
  }

  /*!
      @brief    Sets the brightness of the LED backpack.
      @param    b
                  The brightness value, from 0 (off) to 15 (full brightness).
  */
  virtual void SetLedBackpackBrightness(uint8_t b) {
    // noop
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
  virtual void ConfigureSSD1306(uint8_t width, uint8_t height,
                                uint8_t text_size) {
    // noop
  }

  /*!
      @brief    Writes a message to the SSD1306 display.
      @param    message
                  The message to be displayed.
  */
  virtual void WriteMessageSSD1306(const char *message) {
    // noop
  }
};
#endif // DRV_OUTPUT_BASE_H