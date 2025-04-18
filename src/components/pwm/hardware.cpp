/*!
 * @file src/components/pwm/hardware.cpp
 *
 * Hardware for the pwm.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "hardware.h"

PWMHardware::PWMHardware() {
_is_attached = false;
}

PWMHardware::~PWMHardware() {
  if (_is_attached)
    DetachPin();
}

bool PWMHardware::AttachPin(uint8_t pin, uint32_t frequency, uint32_t resolution) {
#ifdef ARDUINO_ARCH_ESP32
  _is_attached = ledcAttach(_pin, _frequency, _resolution);
#else
  _is_attached = true;
#endif

if (_is_attached) {
  _pin = pin;
  _frequency = frequency;
  _resolution = resolution;
  _duty_cycle = 0;
}

return _is_attached;
}

bool PWMHardware::DetachPin() {
    if (! _is_attached) {
        WS_DEBUG_PRINTLN("[pwm] Pin not attached!");
        return false;
    }
    bool did_detach = false;
    #ifdef ARDUINO_ARCH_ESP32
    did_detach = ledcDetach(_pin);
    #else
    did_detach = true;
    #endif

    // "Disable" the pin's output
    digitalWrite(_pin, LOW);
}

bool PWMHardware::WriteDutyCycle(uint32_t duty) {
  if (! _is_attached) {
    WS_DEBUG_PRINTLN("[pwm] Pin not attached!");
    return false;
  }
  bool did_write = false;
  #ifdef defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH) && defined(STATUS_LED_PIN)
  // Adafruit Feather ESP8266's analogWrite() gets inverted since the builtin
  // LED is reverse-wired
  _duty_cycle = 255 - duty;
  #else
  _duty_cycle = duty;
  #endif

#ifdef ARDUINO_ARCH_ESP32
  did_write = analogWrite(_duty_cycle);
#else
  analogWrite(_pin, duty);
  did_write = true;
#endif

  return true;
}

// LEDC API Wrappers
#ifdef ARDUINO_ARCH_ESP32
bool PWMHardware::analogWrite(uint32_t value) {
  // clamp
  if (value > 255 || value < 0) {
    WS_DEBUG_PRINTLN("[pwm] Value out of range!");
    return false;
  }

  // Calculate duty cycle for the `value` passed in
  // (assumes 12-bit resolution, 2^12)
  uint32_t dutyCycle = (uint32_t)((4095.0 / 255.0) * min(value, (uint32_t)255));
  return ledcWrite(_pin, dutyCycle);
}

#endif // ARDUINO_ARCH_ESP32