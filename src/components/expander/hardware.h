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
  virtual bool begin(uint8_t i2c_addr, TwoWire *wire) = 0;
  uint8_t getAddress() const { return _i2c_addr; }
  virtual void pinMode(uint8_t pin, uint8_t mode) { };
  virtual void digitalWrite(uint8_t pin, uint8_t value) { };
  virtual uint8_t digitalRead(uint8_t pin) { return 0; }
  virtual uint16_t analogRead(uint8_t pin) { return 0; }
  virtual uint8_t getAdcResolution() { return 0; }
  virtual bool setGain(uint8_t gain) { return false; }
  virtual void analogWrite(uint8_t pin, uint16_t value) {};

protected:
  uint8_t _i2c_addr = 0; ///< I2C address of the expander
};

#endif // WS_EXPANDER_HARDWARE_H
