/*!
 * @file src/components/display/drivers/dispDrvBaseI2c.h
 *
 * Base class for I2C-attached display drivers.
 *
 * Holds the I2C bus, address, MUX channel and driver name, and exposes the
 * I2C-specific display API (WriteMessage / WriteValue / Configure*) that the
 * concrete dispDrv* I2C drivers override. Implements the dispDrvBase
 * writeMessage() contract by delegating to WriteMessage(), so the display
 * controller can manage I2C and non-I2C displays through the same pointer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISP_DRV_BASE_RGB666_H
#define WS_DISP_DRV_BASE_RGB666_H

#include "dispDrvBase.h"
#include <Arduino.h>
#include <Wire.h>
#include <protos/display.pb.h>
#include <protos/i2c.pb.h>

/*!
    @brief  Base class for DotClock RGB666-attached display drivers.
*/
class dispDrvBaseRgb666 : public dispDrvBase {
public:
  /*!
      @brief  Constructor.
      @param  descriptor   The TTL RGB666 pin descriptor.
      @param  driver_name  The name of the driver.
      @param  panel        The panel/variant name.
  */
  dispDrvBaseRgb666(const char *driver_name, const char *panel)
      : dispDrvBase() {
    strncpy(_name, driver_name ? driver_name : "", sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    strncpy(_panel, panel ? panel : "", sizeof(_panel) - 1);
    _panel[sizeof(_panel) - 1] = '\0';
  }

  /*! @brief  Virtual destructor. */
  virtual ~dispDrvBaseRgb666() {}

  /*!
      @brief    Gets the name of the driver.
      @returns  The driver's name.
  */
  const char *GetDrvName() { return _name; }

  /*!
      @brief    Gets the name of the panel.
      @returns  The panel's name.
  */
  const char *GetPanelName() { return _panel; }

  /*!
      @brief    Writes a message to the I2C display.
      @param    message  The message to be displayed.
  */
  virtual void WriteMessage(const char *message) {
    // noop
  }

  /*!
      @brief    Writes a floating-point value to the I2C display.
      @param    value  The value to be displayed.
  */
  virtual void WriteValue(float value) {
    // noop
  }

  /*!
      @brief    Writes an integer value to the I2C display.
      @param    value  The value to be displayed.
  */
  virtual void WriteValue(int32_t value) {
    // noop
  }

  /*!
      @brief    Configures a LED backpack.
      @param    brightness  The brightness of the LED backpack.
      @param    alignment   The alignment of the LED backpack.
  */
  virtual void ConfigureI2CBackpack(int32_t brightness, uint32_t alignment) {
    // noop
  }

  /*!
      @brief    Configures a character LCD.
      @param    rows              The number of rows in the LCD.
      @param    cols              The number of columns in the LCD.
      @param    enable_backlight  True to enable the backlight.
  */
  virtual void ConfigureCharLcd(uint32_t rows, uint32_t cols,
                                bool enable_backlight) {
    // noop
  }

  /*!
      @brief    Enables or disables the backlight on a character LCD.
      @param    enable_backlight  True to enable the backlight, False to
                                  disable it.
  */
  virtual void EnableBackLightCharLCD(bool enable_backlight) {
    // noop
  }

  /*!
      @brief    Writes a message to a character LCD via a display Write proto.
      @param    write_msg  Pointer to a ws_display_Write message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool WriteMessageCharLCD(ws_display_Write *write_msg) {
    WriteMessage(write_msg->message);
    return true;
  }

  /*!
      @brief    Sets the brightness of the LED backpack.
      @param    b  Brightness value, from 0 (off) to 15 (full brightness).
  */
  virtual void SetLedBackpackBrightness(uint8_t b) {
    // noop
  }

  /*!
      @brief    Configures an SSD1306-style OLED display. Must be called before
                begin().
      @param    width      The width of the display in pixels.
      @param    height     The height of the display in pixels.
      @param    text_size  The magnification factor for the text size.
  */
  virtual void ConfigureSSD1306(uint8_t width, uint8_t height,
                                uint8_t text_size) {
    // noop
  }

protected:
  char _name[65] = {0};  ///< Driver name
  char _panel[33] = {0}; ///< Panel name
};

#endif // WS_DISP_DRV_BASE_RGB666_H
