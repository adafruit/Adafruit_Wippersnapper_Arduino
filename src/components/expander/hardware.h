/*!
 * @file src/components/expander/hardware.h
 *
 * Base class hardware abstraction for WipperSnapper's I/O expander
 * component. Mirrors the Arduino/Wiring GPIO API so that digitalIO
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

/*!
    @brief  Base class for I/O expander hardware drivers.
            Provides a virtual Arduino/Wiring-style GPIO interface
            so higher layers don't need to know the chip.
*/
class ExpanderHardware {
public:
  virtual ~ExpanderHardware();
  /*!  @brief  Initializes the expander hardware.
       @param  i2c_addr  The I2C address of the expander.
       @param  wire      Pointer to the TwoWire I2C bus instance.
       @return True if initialization succeeded, false otherwise. */
  virtual bool begin(uint8_t i2c_addr, TwoWire *wire) = 0;
  /*!  @brief  Returns the I2C address of the expander.
       @return The I2C address. */
  uint8_t getAddress() const { return _i2c_addr; }
  /*!  @brief  Sets the mode of a pin on the expander.
       @param  pin  The pin number.
       @param  mode The mode (INPUT, OUTPUT, etc.). */
  virtual void pinMode(uint8_t pin, uint8_t mode) {};
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
  virtual uint16_t analogRead(uint8_t pin) { return 0; }
  /*!  @brief  Returns the ADC resolution of the expander in bits.
       @return The ADC resolution. */
  virtual uint8_t getAdcResolution() { return 0; }
  /*!  @brief  Sets the ADC gain of the expander.
       @param  gain The gain value.
       @return True if the gain was set successfully. */
  virtual bool setGain(uint8_t gain) { return false; }
  /*!  @brief  Writes an analog (PWM) value to a pin on the expander.
       @param  pin   The pin number.
       @param  value The PWM value. */
  virtual void analogWrite(uint8_t pin, uint16_t value) {};

protected:
  uint8_t _i2c_addr = 0; ///< I2C address of the expander
};

#endif // WS_EXPANDER_HARDWARE_H
