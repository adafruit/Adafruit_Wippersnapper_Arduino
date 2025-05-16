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
    if (msg_write->adjust_brightness)
      SetLedBackpackBrightness((uint8_t)msg_write->brightness);

    WriteMessage(msg_write->message.text);
    return true;
  }

  /*!
      @brief    Configures a character LCD.
      @param    rows
                  The number of rows in the LCD.
      @param    cols
                  The number of columns in the LCD.
  */
  virtual void ConfigureCharLcd(uint32_t rows, uint32_t cols) {
    // noop
  }

  /*!
      @brief    Turns the character LCD backlight on or off.
      @param    enable
                  True to enable the backlight, false to disable it.
  */
  void EnableCharLcdBacklight(bool enable) {
    // noop
  }

  /*!
      @brief    Writes a message to the LCD.
      @param    write_char_lcd
                Points to a CharLCDWrite message.
      @param    enable_backlight
                True if the backlight should be enabled, false otherwise.
  */
  void WriteMessageCharLCD(wippersnapper_i2c_v1_CharLcdWrite *write_char_lcd,
                           bool enable_backlight = true) {
    EnableCharLcdBacklight(enable_backlight);
    WriteMessage(write_char_lcd->message);
  }
};

#endif // WIPPERSNAPPER_I2C_DRIVER_OUT_H
