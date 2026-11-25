/*!
 * @file src/components/expander/drivers/drv_pca9554.h
 *
 * Header-only expander driver for the PCA9554 / PCA9554A (and TCA9554)
 * 8-pin I2C I/O expander.
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
#include <Adafruit_I2CDevice.h>

/*!
    @brief  Expander driver for the PCA9554 / PCA9554A (8 GPIO pins, I2C).
    @note   Talks to the chip through Adafruit BusIO rather than the
            Adafruit_XCA9554 library, because that library's begin() sets
            every pin to an input. This driver never resets the chip, and
            every change is a read-modify-write of a single bit, so it
            leaves pins it isn't using alone.
    @note   On the Qualia ESP32-S3 the on-board PCA9554A at 0x3F is shared
            with the RGB666 display driver (Arduino_XCA9554SWSPI), which
            uses P0-P2/P7 for the panel's init sequence and P4 for the
            backlight. That library resets the chip's registers in begin()
            and writes the whole output register from its own cache, so on
            a shared chip only inputs (the UP/DN buttons on P5/P6) are
            reliable: outputs set here can be reset when a display is added
            or removed.
    @note   Because the chip is never reset, its registers survive an MCU
            reset. A pin left as an output by earlier firmware stays driven
            until it is configured again.
    @note   Inputs have a fixed internal ~100k pull-up and there is no
            pull-down, so INPUT_PULLUP is treated as INPUT, and a pull-down
            request also leaves the pin as a plain (pulled-up) input.
*/
class ExpanderPCA9554 : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the PCA9554 expander driver.
   */
  ExpanderPCA9554() {}

  /*!
   * @brief Destructor for the PCA9554 expander driver. Leaves the chip's
   *        registers as they are, since other drivers may share it.
   */
  ~ExpanderPCA9554() {
    delete _i2c_dev;
    _i2c_dev = nullptr;
  }

  /*!
   * @brief Initializes the PCA9554 driver with the given I2C address and bus.
   *        Only checks that the chip answers; no registers are written.
   * @param i2c_addr The I2C address of the expander.
   * @param wire The TwoWire I2C bus to use for communication.
   * @return True if the expander responded, False otherwise.
   */
  bool begin(uint8_t i2c_addr, TwoWire *wire) override {
    _i2c_addr = i2c_addr;
    delete _i2c_dev;
    _i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
    if (!_i2c_dev->begin())
      return false;
    // Seed the input cache so a failed first read doesn't look like a press.
    return readReg(REG_INPUT, _last_input);
  }

  /*!
   * @brief Sets the mode of a pin. OUTPUT clears its configuration bit;
   *        every other mode sets it (input) and clears the pin's polarity
   *        inversion bit, which may have been left set by earlier firmware.
   * @param pin The pin number (0-7) to configure.
   * @param mode The mode to set (INPUT, INPUT_PULLUP, OUTPUT, etc.).
   */
  void pinMode(uint8_t pin, uint8_t mode) override {
    bool input = (mode != OUTPUT);
    if (input)
      updateBit(REG_POLARITY, pin, false);
    updateBit(REG_CONFIG, pin, input);
  }

  /*!
   * @brief Writes a digital value to a pin's output register bit.
   * @param pin The pin number (0-7) to write to.
   * @param value The value to write (HIGH or LOW).
   */
  void digitalWrite(uint8_t pin, uint8_t value) override {
    updateBit(REG_OUTPUT, pin, value != LOW);
  }

  /*!
   * @brief Reads a pin's level from the input port register. If the read
   *        fails, returns the pin's level from the last good read, so a bus
   *        glitch doesn't register as a (pull-to-ground) button press.
   * @param pin The pin number (0-7) to read from.
   * @return HIGH or LOW. LOW if the pin is out of range.
   */
  uint8_t digitalRead(uint8_t pin) override {
    if (pin > 7)
      return LOW;
    uint8_t value;
    if (readReg(REG_INPUT, value))
      _last_input = value;
    return (_last_input >> pin) & 0x01 ? HIGH : LOW;
  }

private:
  static const uint8_t REG_INPUT = 0x00;    ///< Input port register
  static const uint8_t REG_OUTPUT = 0x01;   ///< Output port register
  static const uint8_t REG_POLARITY = 0x02; ///< Polarity inversion register
  static const uint8_t REG_CONFIG = 0x03;   ///< Configuration, 1 = input

  /*!
   * @brief Reads one register from the expander.
   * @param reg The register address.
   * @param value Out: the register value.
   * @return True if the read succeeded, False otherwise.
   */
  bool readReg(uint8_t reg, uint8_t &value) {
    if (_i2c_dev == nullptr)
      return false;
    return _i2c_dev->write_then_read(&reg, 1, &value, 1);
  }

  /*!
   * @brief Sets or clears one bit of a register, reading the current value
   *        from the chip first so other pins are left unchanged.
   * @param reg The register address.
   * @param pin The pin (bit) number, 0-7.
   * @param set True to set the bit, False to clear it.
   */
  void updateBit(uint8_t reg, uint8_t pin, bool set) {
    uint8_t value;
    if (pin > 7 || !readReg(reg, value))
      return;
    uint8_t updated = set ? (value | (1 << pin)) : (value & ~(1 << pin));
    if (updated == value)
      return;
    uint8_t buf[2] = {reg, updated};
    _i2c_dev->write(buf, 2);
  }

  Adafruit_I2CDevice *_i2c_dev = nullptr; ///< BusIO I2C device for the chip
  uint8_t _last_input = 0xFF; ///< Last good input port read (pulled-up idle)
};

#endif // WS_EXPANDER_DRV_PCA9554_H
