/*!
 * @file src/components/expander/drivers/drv_seesaw.cpp
 *
 * Expander driver for the Adafruit Seesaw expander.
 * Wraps the Adafruit_Seesaw library.
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
#include "drv_seesaw.h"
#include <Adafruit_seesaw.h>

/*!
    @brief  Constructor for the Seesaw expander driver.
*/
ExpanderSeesaw::ExpanderSeesaw() {
  _resolution = DEFAULT_SEESAW_ADC_RESOLUTION;
}

/*!
    @brief  Destructor for the Seesaw expander driver.
*/
ExpanderSeesaw::~ExpanderSeesaw() {
  delete _ss;
  _ss = nullptr;
}

/*!
    @brief  Initializes the Seesaw driver with the given I2C address and bus.
    @param  i2c_addr  The I2C address of the expander.
    @param  wire      The TwoWire I2C bus to use for communication.
    @return True if initialization was successful, False otherwise.
*/
bool ExpanderSeesaw::begin(uint8_t i2c_addr, TwoWire *wire) {
  _i2c_addr = i2c_addr;
  delete _ss;
  _ss = new Adafruit_seesaw(wire);
  return _ss->begin(i2c_addr);
}

/*!
    @brief  Sets the mode of a specific pin on the Seesaw.
    @note   The mode is the core's INPUT/INPUT_PULLUP/INPUT_PULLDOWN value.
            The seesaw library is built against the same value, so it
            passes through unchanged.
    @param  pin   The pin number to configure.
    @param  mode  The mode to set (INPUT, OUTPUT, etc.).
*/
void ExpanderSeesaw::pinMode(uint8_t pin, uint8_t mode) {
  if (_ss != nullptr)
    _ss->pinMode(pin, mode);
}

/*!
    @brief  Writes a digital value to a specific pin on the Seesaw.
    @param  pin    The pin number to write to.
    @param  value  The value to write (HIGH or LOW).
*/
void ExpanderSeesaw::digitalWrite(uint8_t pin, uint8_t value) {
  if (_ss != nullptr)
    _ss->digitalWrite(pin, value);
}

/*!
    @brief  Reads a digital value from a specific pin on the Seesaw.
    @param  pin  The pin number to read from.
    @return The digital value read (HIGH or LOW).
*/
uint8_t ExpanderSeesaw::digitalRead(uint8_t pin) {
  return _ss != nullptr ? _ss->digitalRead(pin) : LOW;
}

/*!
    @brief  Gets the ADC raw value for a given pin/ADC channel.
    @param  pin  GPIO pin to read analog value
    @return Analog raw value (non-calibrated).
*/
uint32_t ExpanderSeesaw::analogRead(uint8_t pin) {
  return _ss != nullptr ? (uint32_t)_ss->analogRead(pin) : 0;
}

/*!
    @brief  Gets the resolution of the seesaw ADC.
    @return ADC resolution, in bits.
*/
uint8_t ExpanderSeesaw::getAdcResolution() { return _resolution; }

/*!
    @brief  Writes an analog value (PWM waveform) to a pin on the Seesaw.
    @param  pin    The pin number to write to.
    @param  value  The duty cycle value to write.
*/
void ExpanderSeesaw::analogWrite(uint8_t pin, uint16_t value) {
  if (_ss != nullptr)
    _ss->analogWrite(pin, value);
}
