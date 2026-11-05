/*!
 * @file src/components/expander/drivers/drv_seesaw.h
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
#ifndef WS_EXPANDER_DRV_SEESAW_H
#define WS_EXPANDER_DRV_SEESAW_H
#include "../hardware.h"

// Only include Adafruit_seesaw.h from drv_seesaw.cpp: it #defines
// INPUT_PULLDOWN as 0x03 when the core has no such macro (e.g. 0x03 is
// OUTPUT_OPEN_DRAIN on ESP8266), which would leak into every file that
// includes this header.
class Adafruit_seesaw;

#define DEFAULT_SEESAW_ADC_RESOLUTION                                          \
  10 ///< Default ADC resolution for the Seesaw, in bits

/*!
    @brief  Expander driver for the Adafruit Seesaw (various GPIO pins, I2C).
*/
class ExpanderSeesaw : public ExpanderHardware {
public:
  ExpanderSeesaw();
  ~ExpanderSeesaw();
  bool begin(uint8_t i2c_addr, TwoWire *wire) override;
  void pinMode(uint8_t pin, uint8_t mode) override;
  void digitalWrite(uint8_t pin, uint8_t value) override;
  uint8_t digitalRead(uint8_t pin) override;
  uint32_t analogRead(uint8_t pin) override;
  uint8_t getAdcResolution() override;
  void analogWrite(uint8_t pin, uint16_t value) override;

private:
  Adafruit_seesaw *_ss = nullptr; ///< Adafruit Seesaw driver instance
  uint8_t _resolution;            ///< ADC resolution, in bits
};

#endif // WS_EXPANDER_DRV_SEESAW_H
