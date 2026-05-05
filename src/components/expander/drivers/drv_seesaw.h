/*!
 * @file src/components/expander/drivers/drv_seesaw.h
 *
 * Header-only expander driver for the Adafruit Seesaw expander.
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
#ifndef WS_EXPANDER_DRV_SEESAW_H
#define WS_EXPANDER_DRV_SEESAW_H
#include "../hardware.h"
#include <Adafruit_seesaw.h>

/*!
    @brief  Expander driver for the Adafruit Seesaw (various GPIO pins, I2C).
*/
class ExpanderSeesaw : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the Seesaw expander driver.
   */
  ExpanderSeesaw() {}

  /*!
   * @brief Destructor for the Seesaw expander driver.
   */
  ~ExpanderSeesaw() {}

  /*!
   * @brief Initializes the Seesaw driver with the given I2C address and bus.
   * @param i2c_addr The I2C address of the expander.
   * @param wire The TwoWire I2C bus to use for communication.
   * @return True if initialization was successful, False otherwise.
   */
  bool begin(uint8_t i2c_addr, TwoWire *wire) override {
    _i2c_addr = i2c_addr;
    _ss = Adafruit_seesaw(wire);
    return _ss.begin(i2c_addr);
  }

  /*!
   * @brief Sets the mode of a specific pin on the Seesaw.
   * @param pin The pin number to configure.
   * @param mode The mode to set (INPUT, OUTPUT, etc.).
   */
  void pinMode(uint8_t pin, uint8_t mode) override { _ss.pinMode(pin, mode); }

  /*!
   * @brief Writes a digital value to a specific pin on the Seesaw.
   * @param pin The pin number to write to.
   * @param value The value to write (HIGH or LOW).
   */
  void digitalWrite(uint8_t pin, uint8_t value) override {
    _ss.digitalWrite(pin, value);
  }

  /*!
   * @brief Reads a digital value from a specific pin on the TCA8418.
   * @param pin The pin number to read from.
   * @return The digital value read (HIGH or LOW).
   */
  uint8_t digitalRead(uint8_t pin) override { return _ss.digitalRead(pin); }


  /*!
   * @brief This function is used to get the ADC raw value for a given pin/ADC channel.
   * @param pin GPIO pin to read analog value
   * @return Analog raw value (non-calibrated).
   */
  uint16_t analogRead(uint8_t pin) override { return _ss.analogRead(pin); }
private:
  Adafruit_seesaw _ss; ///< Adafruit Seesaw driver instance
};

#endif // WS_EXPANDER_DRV_SEESAW_H
