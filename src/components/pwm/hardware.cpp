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

/**************************************************************************/
/*!
    @brief  Ctor for PWMHardware
*/
/**************************************************************************/
PWMHardware::PWMHardware() {
_is_attached = false;
}

/**************************************************************************/
/*!
    @brief  Dtor for PWMHardware
*/
/**************************************************************************/
PWMHardware::~PWMHardware() {
  if (_is_attached)
    DetachPin();
}

/**************************************************************************/
/*!
    @brief  Attach a pin to the PWM hardware
    @param  pin The pin to attach
    @param  frequency The frequency of the PWM signal
    @param  resolution The resolution of the PWM signal
    @return true if the pin was successfully attached, false otherwise
*/
/**************************************************************************/
bool PWMHardware::AttachPin(uint8_t pin, uint32_t frequency, uint32_t resolution) {
#ifdef ARDUINO_ARCH_ESP32
  _is_attached = ledcAttach(pin, frequency, resolution);
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

/**************************************************************************/
/*!
    @brief  Detaches a PWM pin and frees it for use.
    @return true if the PWM pin was successfully detached, false otherwise.
*/
/**************************************************************************/
bool PWMHardware::DetachPin() {
    if (! _is_attached) {
        WS_DEBUG_PRINTLN("[pwm] Pin not attached!");
        return false;
    }
    bool did_detach = false;
    #ifdef ARDUINO_ARCH_ESP32
    did_detach = ledcDetach(_pin);
    #else
    digitalWrite(_pin, LOW); // "Disable" the pin's output
    did_detach = true;
    #endif

    _is_attached = false; // always mark as false, for tracking
    return did_detach;
}

/**************************************************************************/
/*!
    @brief  Writes a duty cycle to a PWM pin with a fixed frequency
            of 5kHz and 8-bit resolution.
    @param  duty The desired duty cycle to write to the pin.
    @return true if the duty cycle was successfully written, false otherwise
*/
/**************************************************************************/
bool PWMHardware::WriteDutyCycle(uint32_t duty) {
  if (! _is_attached) {
    WS_DEBUG_PRINTLN("[pwm] Pin not attached!");
    return false;
  }
  bool did_write = false;
  #if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH) && defined(STATUS_LED_PIN)
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

/**************************************************************************/
/*!
    @brief  Writes a frequency to a PWM pin with a fixed duty cycle.
    @param  freq The desired frequency to write to the pin.
    @return The frequency that was written to the pin.
*/
/**************************************************************************/
uint32_t PWMHardware::WriteTone(uint32_t freq) {
  if (! _is_attached) {
    WS_DEBUG_PRINTLN("[pwm] Pin not attached!");
    return false;
  }

  uint32_t rc = 0;
  #ifdef ARDUINO_ARCH_ESP32
  rc = ledcWriteTone(_pin, freq);
  #else
  tone(_pin, freq);
  rc = freq;
  #endif

  return rc;
}

/**************************************************************************/
/*!
    @brief  Returns the pin number of the PWM pin
    @return The logical pin number of the PWM pin
*/
/**************************************************************************/
uint8_t PWMHardware::GetPin() {
    return _pin;
}

// LEDC API Wrappers
#ifdef ARDUINO_ARCH_ESP32
/**************************************************************************/
/*!
    @brief  Mocks the Arduino analogWrite() function for the Arduino-ESP32
            LEDC API
    @param  value The value to write (0-255)
    @return true if the value was successfully written, false otherwise
*/
/**************************************************************************/
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