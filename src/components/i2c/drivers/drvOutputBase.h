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
#include <protos/display.pb.h>

// Shared LED backpack defines (used by drvOut7Seg and drvOutQuadAlphaNum)
#ifndef LED_BACKPACK_ALIGNMENT_UNSPECIFIED
#define LED_BACKPACK_ALIGNMENT_UNSPECIFIED 0
#define LED_BACKPACK_ALIGNMENT_LEFT 1
#define LED_BACKPACK_ALIGNMENT_RIGHT 2
#define LED_BACKPACK_ALIGNMENT_DEFAULT LED_BACKPACK_ALIGNMENT_LEFT
#define LED_MAX_CHARS 4
#endif

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
      @brief    Writes a message to the LCD using a display Write message.
      @param    write_msg
                Pointer to a ws_display_Write message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool WriteMessageCharLCD(ws_display_Write *write_msg) {
    if (write_msg->which_content == ws_display_Write_message_tag) {
      WriteMessage(write_msg->content.message);
    }
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

protected:
  /*! 
      @brief  Parses a display-write token at the given index.
      @param  message       Input message buffer.
      @param  msg_size      Message length.
      @param  idx           Current index (updated when a multi-byte token is consumed).
      @param  out_char      Parsed output character when token is a glyph.
      @param  is_newline    Set true when token is a newline marker.
      @param  degree_char   Driver-specific glyph for the degree symbol.
      @return True when a token was recognized and consumed.
  */
  bool ParseWriteToken(const char *message, size_t msg_size, size_t &idx,
                       char &out_char, bool &is_newline,
                       char degree_char = char(247)) const {
    out_char = 0;
    is_newline = false;

    if (!message || idx >= msg_size)
      return false;

    // Handle escaped CRLF and LF sequences ("\\r\\n", "\\n").
    if (message[idx] == '\\' && idx + 1 < msg_size) {
      if (message[idx + 1] == 'r' && idx + 3 < msg_size &&
          message[idx + 2] == '\\' && message[idx + 3] == 'n') {
        idx += 3;
        is_newline = true;
        return true;
      }
      if (message[idx + 1] == 'n') {
        idx += 1;
        is_newline = true;
        return true;
      }
    }

    // Handle literal CRLF, CR, and LF.
    if (message[idx] == '\r') {
      if (idx + 1 < msg_size && message[idx + 1] == '\n') {
        idx += 1;
      }
      is_newline = true;
      return true;
    }
    if (message[idx] == '\n') {
      is_newline = true;
      return true;
    }

    // Handle UTF-8 degree symbol (0xC2 0xB0).
    if ((uint8_t)message[idx] == 0xC2 && idx + 1 < msg_size &&
        (uint8_t)message[idx + 1] == 0xB0) {
      idx += 1;
      out_char = degree_char;
      return true;
    }

    return false;
  }
};
#endif // DRV_OUTPUT_BASE_H