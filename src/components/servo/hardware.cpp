/*!
 * @file src/components/servo/hardware.cpp
 *
 * Hardware for the servo.proto message.
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
    @brief  Constructs a ServoHardware object
    @param  pin
            The GPIO pin to attach the servo to
    @param  min_pulse_width
            The minimum pulse width, in microseconds
    @param  max_pulse_width
            The maximum pulse width, in microseconds
    @param  frequency
            The frequency of the PWM signal, in Hz (50Hz is sent from IO)
*/
/**************************************************************************/
ServoHardware::ServoHardware(int pin, int min_pulse_width, int max_pulse_width,
                             int frequency) {
  _pin = pin;
  _min_pulse_width = min_pulse_width;
  _max_pulse_width = max_pulse_width;
  _frequency = frequency;

#ifndef ARDUINO_ARCH_ESP32
  _servo = new Servo();
#else
  _is_attached = false;
#endif
}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ServoHardware::~ServoHardware() {}

/**************************************************************************/
/*!
    @brief  Attempts to attach a servo to a GPIO pin.
    @returns true if successful, false otherwise.
*/
/**************************************************************************/
bool ServoHardware::ServoAttach() {
  uint16_t rc = 255;

// Attach the servo to the pin
#ifdef ARDUINO_ARCH_ESP32
  if (!ledcAttach(_pin, _frequency, LEDC_TIMER_WIDTH)) {
    rc = 255;
  } else {
    rc = 1;
    _is_attached = true;
  }
#else
  rc = _servo.attach(_pin, _min_pulse_width, _max_pulse_width);
#endif

  if (rc == 255) {
    WS_DEBUG_PRINT("[servo] Error: Failed to attach servo to pin: ");
    WS_DEBUG_PRINTLN(_pin);
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Writes a value to the servo pin
    @param  value
            The value to write to the servo pin
*/
/**************************************************************************/
void ServoHardware::ServoWrite(int value) {
#ifdef ARDUINO_ARCH_ESP32
  writeMicroseconds(value);
#else
  _servo.writeMicroseconds(value);
#endif
}

#ifdef ARDUINO_ARCH_ESP32
/**************************************************************************/
/*!
    @brief  Mocks writeMicroseconds() call in arduino/servo api for
            ESP32x's LEDC manager.
    @param  value
            The value to write to the servo pin.
*/
/**************************************************************************/
void ServoHardware::writeMicroseconds(int value) {
  if (!_is_attached) {
    WS_DEBUG_PRINTLN("[servo] Error: Servo not attached!");
    return;
  }

  // Clamp value to a valid pulse_width range
  if (value < _min_pulse_width)
    value = _min_pulse_width;
  if (value > _max_pulse_width)
    value = _max_pulse_width;

  // Formula from ESP32Servo library
  // https://github.com/madhephaestus/ESP32Servo/blob/master/src/ESP32Servo.cpp
  // count = (pulse_high_width / (pulse_period/2**timer_width))
  // 50Hz servo =  20ms pulse_period
  uint32_t count =
      ((double)value / ((double)20000 / (double)pow(2, LEDC_TIMER_WIDTH)));
  if (!ledcWrite(_pin, count))
    WS_DEBUG_PRINTLN("[servo] Error: Failed to write to servo pin!");
}
#endif