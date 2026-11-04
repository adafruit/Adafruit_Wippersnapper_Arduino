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

#define DEFAULT_SEESAW_ADC_RESOLUTION                                          \
  10 ///< Default ADC resolution for the Seesaw, in bits

/*!
    @brief  Adafruit_seesaw with the chip type it detected in begin() made
            readable (the library keeps it protected).
*/
class SeesawWithChipType : public Adafruit_seesaw {
public:
  /*!
   * @brief Constructor.
   * @param wire The TwoWire I2C bus to use for communication.
   */
  SeesawWithChipType(TwoWire *wire = &Wire) : Adafruit_seesaw(wire) {}

  /*!
   * @brief Gets the chip type detected in begin().
   * @return A SEESAW_HW_ID_CODE_* value, or 0 before begin().
   */
  uint8_t getHardwareType() const { return _hardwaretype; }
};

/*!
    @brief  Expander driver for the Adafruit Seesaw (various GPIO pins, I2C).
*/
class ExpanderSeesaw : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the Seesaw expander driver.
   */
  ExpanderSeesaw() { _resolution = DEFAULT_SEESAW_ADC_RESOLUTION; }

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
    _ss = SeesawWithChipType(wire);
    return _ss.begin(i2c_addr);
  }

  /*!
   * @brief Sets the mode of a specific pin on the Seesaw.
   * @param pin The pin number to configure.
   * @param mode OUTPUT, or a WS_INPUT* mode from ws_boards.h.
   */
  void pinMode(uint8_t pin, uint8_t mode) override {
#if defined(WS_INPUT_PULLDOWN)
    // The seesaw library compares against its own INPUT_PULLDOWN, which it
    // #defines as 0x03 when the core has none (e.g. ESP8266)
    if (mode == WS_INPUT_PULLDOWN)
      mode = INPUT_PULLDOWN;
#endif
    _ss.pinMode(pin, mode);
  }

  /*!
   * @brief Reports whether a pin has an internal pull resistor. Every seesaw
   *        GPIO pin has a pull-up; only SAMD09 and STM32 chips also have
   *        pull-downs (tinyAVR has pull-ups only).
   * @param pin The pin number.
   * @param pull_up True to check for a pull-up, False for a pull-down.
   * @return True if the pin has the requested pull resistor.
   */
  bool hasPull(uint8_t pin, bool pull_up) override {
    (void)pin;
    if (pull_up)
      return true;
    switch (_ss.getHardwareType()) {
    case SEESAW_HW_ID_CODE_SAMD09:
#ifdef SEESAW_HW_ID_CODE_STM32C011
    case SEESAW_HW_ID_CODE_STM32C011:
#endif
#ifdef SEESAW_HW_ID_CODE_STM32C031
    case SEESAW_HW_ID_CODE_STM32C031:
#endif
#ifdef SEESAW_HW_ID_CODE_STM32C051
    case SEESAW_HW_ID_CODE_STM32C051:
#endif
#ifdef SEESAW_HW_ID_CODE_STM32G071
    case SEESAW_HW_ID_CODE_STM32G071:
#endif
      return true;
    default:
      return false;
    }
  }

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
   * @brief Gets the ADC raw value for a given pin/ADC channel.
   * @param pin GPIO pin to read analog value
   * @return Analog raw value (non-calibrated).
   */
  uint32_t analogRead(uint8_t pin) override {
    return (uint32_t)_ss.analogRead(pin);
  }

  /*!
   * @brief Gets the resolution of the seesaw ADC.
   * @return ADC resolution, in bits.
   */
  uint8_t getAdcResolution() override { return _resolution; }

  /*!
   * @brief Writes an analog value (PWM waveform) to a pin on the Seesaw.
   * @param pin The pin number to write to.
   * @param value The duty cycle value to write.
   */
  void analogWrite(uint8_t pin, uint16_t value) override {
    _ss.analogWrite(pin, value);
  }

private:
  SeesawWithChipType _ss; ///< Adafruit Seesaw driver instance
  uint8_t _resolution;
};

#endif // WS_EXPANDER_DRV_SEESAW_H
