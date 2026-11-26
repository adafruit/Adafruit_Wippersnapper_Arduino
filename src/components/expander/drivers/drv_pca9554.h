/*!
 * @file src/components/expander/drivers/drv_pca9554.h
 *
 * Header-only expander driver for the PCA9554 / PCA9554A (and TCA9554)
 * 8-pin I2C I/O expander. Wraps the Adafruit_XCA9554 library.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_EXPANDER_DRV_PCA9554_H
#define WS_EXPANDER_DRV_PCA9554_H
#include "../hardware.h"
#include <Adafruit_XCA9554.h>

/*!
    @brief  Expander driver for the PCA9554 / PCA9554A (8 GPIO pins, I2C).
    @note   Adafruit_XCA9554::begin() writes the config register to 0xFF,
            making every pin an input. That is the chip's power-on state,
            but if another driver already set pins as outputs (e.g. the
            Qualia display's backlight on P4), adding or replacing this
            expander afterwards turns them back into pulled-up inputs.
            pinMode() and digitalWrite() are single-bit read-modify-writes,
            so after begin() other pins are left as they are.
    @note   Adafruit_XCA9554 1.0.0 has no destructor, so its I2C device and
            register objects leak each time this driver is deleted (expander
            removed, modified or replaced).
    @note   On the Qualia ESP32-S3 the on-board PCA9554A at 0x3F is shared
            with the RGB666 display driver (Arduino_XCA9554SWSPI), which
            uses P0-P2/P7 for the panel's init sequence and P4 for the
            backlight. That library resets the chip's registers in its
            begin() and writes the whole output register from its own
            cache, so on a shared chip only inputs (the UP/DN buttons on
            P5/P6) are reliable: outputs set here can be reset when a
            display is added or removed.
    @note   Inputs have a fixed internal ~100k pull-up and there is no
            pull-down. The library only accepts INPUT and OUTPUT, so every
            input mode (INPUT_PULLUP, a pull-down request) is passed as
            INPUT and the pin is a pulled-up input.
*/
class ExpanderPCA9554 : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the PCA9554 expander driver.
   */
  ExpanderPCA9554() {}

  /*!
   * @brief Destructor for the PCA9554 expander driver.
   */
  ~ExpanderPCA9554() {}

  /*!
   * @brief Initializes the PCA9554 driver with the given I2C address and bus.
   * @param i2c_addr The I2C address of the expander.
   * @param wire The TwoWire I2C bus to use for communication.
   * @return True if initialization was successful, False otherwise.
   */
  bool begin(uint8_t i2c_addr, TwoWire *wire) override {
    _i2c_addr = i2c_addr;
    return _xca.begin(i2c_addr, wire);
  }

  /*!
   * @brief Sets the mode of a specific pin on the PCA9554. Every mode other
   *        than OUTPUT is passed to the library as INPUT.
   * @param pin The pin number to configure.
   * @param mode The mode to set (INPUT, INPUT_PULLUP, OUTPUT, etc.).
   */
  void pinMode(uint8_t pin, uint8_t mode) override {
    _xca.pinMode(pin, mode == OUTPUT ? OUTPUT : INPUT);
  }

  /*!
   * @brief Writes a digital value to a specific pin on the PCA9554.
   * @param pin The pin number to write to.
   * @param value The value to write (HIGH or LOW).
   */
  void digitalWrite(uint8_t pin, uint8_t value) override {
    _xca.digitalWrite(pin, value != LOW);
  }

  /*!
   * @brief Reads a digital value from a specific pin on the PCA9554.
   * @param pin The pin number to read from.
   * @return The digital value read (HIGH or LOW).
   */
  uint8_t digitalRead(uint8_t pin) override {
    return _xca.digitalRead(pin) ? HIGH : LOW;
  }

private:
  Adafruit_XCA9554 _xca; ///< Adafruit XCA9554 driver instance
};

#endif // WS_EXPANDER_DRV_PCA9554_H
