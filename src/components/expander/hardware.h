/*!
 * @file src/components/expander/hardware.h
 *
 * Base class hardware abstraction for WipperSnapper's I/O expander
 * component. Mirrors the Arduino/Wiring GPIO API so that digital
 * can treat expander pins the same as native pins.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_EXPANDER_HARDWARE_H
#define WS_EXPANDER_HARDWARE_H
#include <Arduino.h>
#include <Wire.h>
#include <protos/config.pb.h>
// WS_INPUT* pin modes, needed by drivers before their own library headers
#include "ws_boards.h"

/*!
    @brief  Base class for I/O expander hardware drivers.
            Provides a virtual Arduino/Wiring-style GPIO interface
            so higher layers don't need to know the chip.
*/
class ExpanderHardware {
public:
  virtual ~ExpanderHardware() {}

  /*!  @brief  Initializes the expander hardware.
       @param  i2c_addr  The I2C address of the expander.
       @param  wire      Pointer to the TwoWire I2C bus instance.
       @return True if initialization succeeded, false otherwise. */
  virtual bool begin(uint8_t i2c_addr, TwoWire *wire) = 0;

  /*!  @brief  Returns the I2C address of the expander.
       @return The I2C address. */
  uint8_t getAddress() const { return _i2c_addr; }

  /*!  @brief  Formats an expander pin name into a buffer.
               Inverse of WsPinNameToNum()/WsPinNameToExpanderAddr() —
               builds "EXP_0xNN_P" from address and pin.
       @param  buf       Output buffer (must be >= 16 bytes).
       @param  buf_size  Size of the output buffer.
       @param  addr      The I2C address of the expander.
       @param  pin_num   The pin number on the expander. */
  static void FormatPinName(char *buf, size_t buf_size, uint8_t addr,
                            uint8_t pin_num) {
    snprintf(buf, buf_size, "EXP_0x%02x_%d", addr, pin_num);
  }

  /*!  @brief  Sets the mode of a pin on the expander.
       @param  pin  The pin number.
       @param  mode OUTPUT, or WS_INPUT / WS_INPUT_PULLUP /
                    WS_INPUT_PULLDOWN (see ws_boards.h). Drivers translate
                    WS_INPUT* modes to their library's constants. */
  virtual void pinMode(uint8_t pin, uint8_t mode) {};

  /*!  @brief  Reports whether a pin has an internal pull resistor. Drivers
               override this to match the hasPullup/hasPulldown flags in
               their Wippersnapper_Components expander definition.json.
               Defaults to no pull resistors.
       @param  pin      The pin number.
       @param  pull_up  True to check for a pull-up, False for a pull-down.
       @return True if pinMode(pin, WS_INPUT_PULLUP / WS_INPUT_PULLDOWN)
               is supported. */
  virtual bool hasPull(uint8_t pin, bool pull_up) {
    (void)pin;
    (void)pull_up;
    return false;
  }

  /*!  @brief  Writes a digital value to a pin on the expander.
       @param  pin   The pin number.
       @param  value HIGH or LOW. */
  virtual void digitalWrite(uint8_t pin, uint8_t value) {};

  /*!  @brief  Reads the digital value of a pin on the expander.
       @param  pin  The pin number.
       @return HIGH or LOW. */
  virtual uint8_t digitalRead(uint8_t pin) { return 0; }

  /*!  @brief  Reads the analog value of a pin on the expander.
       @param  pin  The pin number.
       @return The raw ADC value. */
  virtual uint32_t analogRead(uint8_t pin) { return 0; }

  /*!  @brief  Returns the ADC resolution of the expander in bits.
       @return The ADC resolution. */
  virtual uint8_t getAdcResolution() { return 0; }

  /*!  @brief  Writes an analog (PWM) value to a pin on the expander.
       @param  pin   The pin number.
       @param  value The PWM value. */
  virtual void analogWrite(uint8_t pin, uint16_t value) {};

  /*!  @brief  Applies a gain setting to the expander.
               Override in subclass to apply gain to the hardware driver.
       @param  gain  The decoded gain setting from the broker.
       @return True if applied successfully, false otherwise. */
  virtual bool setGain(const ws_config_Value &gain) { return false; }

protected:
  uint8_t _i2c_addr = 0; ///< I2C address of the expander
};

#endif // WS_EXPANDER_HARDWARE_H
