/*!
 * @file WipperSnapper_I2C_Driver_Out.h
 *
 * Derived class for I2C output driver components
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

#ifndef WIPPERSNAPPER_I2C_DRIVER_OUT_H
#define WIPPERSNAPPER_I2C_DRIVER_OUT_H

#include "WipperSnapper_I2C_Driver.h"
#include <Arduino.h>

/*!
    @brief  Derived class for I2C output component drivers.
*/
class WipperSnapper_I2C_Driver_Out : public WipperSnapper_I2C_Driver {

public:
  /*!
      @brief    Creates a new I2C output component driver.
      @param    i2c
                The I2C hardware interface, default is Wire.
      @param    sensorAddress
                The I2C sensor's unique address.
  */
  WipperSnapper_I2C_Driver_Out(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    // No-op constructor
  }

  /*!
      @brief    Destructor for an I2C output component.
  */
  virtual ~WipperSnapper_I2C_Driver_Out() {
    // No-op destructor
  }

  /*!
    @brief    Writes a message to an i2c output device.
    @param    message
              The message to be displayed.
*/
  virtual void WriteMessage(const char *message) {
    // noop
  }

  /*!
      @brief    Writes a floating point value to an i2c output device.
      @param    value
                  The value to be displayed. Only the first four digits are
     displayed.
  */
  virtual void WriteValue(float value) {
    // noop
  }

  /*!
      @brief    Writes a floating point value to an i2c output device.
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
      @brief    Sets the brightness of the LED backpack.
      @param    b
                  The brightness value, from 0 (off) to 15 (full brightness).
  */
  virtual void SetLedBackpackBrightness(uint8_t b) {
    // noop
  }

  /*!
      @brief    Writes a message to the LED backpack.
      @param    msg_write
                Pointer to a wippersnapper_i2c_v1_LedBackpackWrite message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool WriteLedBackpack(wippersnapper_i2c_v1_LedBackpackWrite *msg_write) {
    // Check if we should adjust brightness
    if (msg_write->adjust_brightness)
      SetLedBackpackBrightness((uint8_t)msg_write->brightness);

    // Write the message to a LED backpack
    switch (msg_write->which_message) {
    case wippersnapper_i2c_v1_LedBackpackWrite_text_tag:
      WS_DEBUG_PRINTLN("[i2c] Writing text to LED backpack...");
      WriteMessage(msg_write->message.text);
      break;
    case wippersnapper_i2c_v1_LedBackpackWrite_number_int_tag:
      WS_DEBUG_PRINTLN("[i2c] Writing int to LED backpack...");
      WriteValue(msg_write->message.number_int);
      break;
    case wippersnapper_i2c_v1_LedBackpackWrite_number_float_tag:
      WS_DEBUG_PRINTLN("[i2c] Writing float to LED backpack...");
      WriteValue(msg_write->message.number_float);
      break;
    default:
      WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to determine message type!");
      return false;
      break;
    }
    return true;
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
      @brief    Writes a message to the LCD.
      @param    write_char_lcd
                Points to a CharLCDWrite message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool WriteMessageCharLCD(wippersnapper_i2c_v1_CharLCDWrite *write_char_lcd) {
    WriteMessage(write_char_lcd->message);
    // NOTE: While this isn't calling any other funcs in here and ret'ing true,
    // I want to keep this function high-level for when we implement backlight
    // color and scrolling.
    return true;
  }
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_H
