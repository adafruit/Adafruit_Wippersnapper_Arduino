/*!
 * @file src/components/expander/drivers/drv_pcf8575.h
 *
 * Header-only expander driver for the PCF8575 16-pin I/O expander.
 * Wraps the Adafruit_PCF8575 library.
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
#ifndef WS_EXPANDER_DRV_PCF8575_H
#define WS_EXPANDER_DRV_PCF8575_H
#include "../hardware.h"
#include <Adafruit_PCF8575.h>

/*!
    @brief  Expander driver for the PCF8575 (16 GPIO pins, I2C).
*/
class ExpanderPCF8575 : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the PCF8575 expander driver.
   */
  ExpanderPCF8575() {}

  /*!
   * @brief Destructor for the PCF8575 expander driver.
   */
  ~ExpanderPCF8575() {}

  /*!
   * @brief Initializes the PCF8575 driver with the given I2C address and bus.
   * @param i2c_addr The I2C address of the expander.
   * @param wire The TwoWire I2C bus to use for communication.
   * @return True if initialization was successful, False otherwise.
   */
  bool begin(uint8_t i2c_addr, TwoWire *wire) override {
    _i2c_addr = i2c_addr;
    return _pcf.begin(i2c_addr, wire);
  }

  /*!
   * @brief Sets the mode of a specific pin on the PCF8575.
   * @param pin The pin number to configure.
   * @param mode The mode to set (INPUT, OUTPUT, etc.).
   */
  void pinMode(uint8_t pin, uint8_t mode) override { _pcf.pinMode(pin, mode); }

  /*!
   * @brief Writes a digital value to a specific pin on the PCF8575.
   * @param pin The pin number to write to.
   * @param value The value to write (HIGH or LOW).
   */
  void digitalWrite(uint8_t pin, uint8_t value) override {
    _pcf.digitalWrite(pin, value);
  }

  /*!
   * @brief Reads a digital value from a specific pin on the PCF8575.
   * @param pin The pin number to read from.
   * @return The digital value read (HIGH or LOW).
   */
  uint8_t digitalRead(uint8_t pin) override { return _pcf.digitalRead(pin); }

private:
  Adafruit_PCF8575 _pcf; ///< Adafruit PCF8575 driver instance
};

#endif // WS_EXPANDER_DRV_PCF8575_H
