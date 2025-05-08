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
      @returns  True if initialized successfully, False otherwise.
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
  virtual void ConfigureLEDBackpack(int32_t brightness, uint32_t alignment) {
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
      @brief    High-level fn, executes a call to the appropriate driver
     function(s) based on the message data type to write.
      @param    msg_backpack_write
                Pointer to a wippersnapper_i2c_output_LedBackpackWrite message.
      @returns  True if the message was written successfully, False otherwise.
  */
  bool LedBackpackWrite(
      wippersnapper_i2c_output_LedBackpackWrite *msg_backpack_write) {
    // Check if we should adjust brightness
    if (msg_backpack_write->adjust_brightness)
      SetLedBackpackBrightness((uint8_t)msg_backpack_write->brightness);

    // Write the message to a LED backpack
    switch (msg_backpack_write->which_message) {
    case wippersnapper_i2c_output_LedBackpackWrite_text_tag:
      WriteMessage(msg_backpack_write->message.text);
      break;
    case wippersnapper_i2c_output_LedBackpackWrite_number_int_tag:
      WriteValue(msg_backpack_write->message.number_int);
      break;
    case wippersnapper_i2c_output_LedBackpackWrite_number_float_tag:
      WriteValue(msg_backpack_write->message.number_float);
      break;
    default:
      return false;
      break;
    }
    return true;
  }

protected:
};
#endif // DRV_OUTPUT_BASE_H