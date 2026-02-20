/*!
 * @file src/components/servo/hardware.h
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
#ifndef WS_SERVO_HARDWARE_H
#define WS_SERVO_HARDWARE_H
#include "wippersnapper.h"

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-ledc.h"
#include "esp_err.h"
#define LEDC_TIMER_WIDTH                                                       \
  SOC_LEDC_TIMER_BIT_WIDTH ///< Dynamically scale bit width for each ESP32-x
                           ///< Arch.
#else
#include <Servo.h>
#endif

#define ERROR_SERVO_ATTACH 255 ///< Error code for servo attach failure

/*!
    @brief  Interface for interacting with hardware's Servo API.
*/
class ServoHardware {
public:
  ServoHardware(int pin, int min_pulse_width, int max_pulse_width,
                int frequency);
  ~ServoHardware();
  bool ServoAttach();
  void ServoWrite(int value);
  uint8_t GetPin();

private:
  bool ServoDetach();
  int ClampPulseWidth(int value);
#ifdef ARDUINO_ARCH_ESP32
  // Mocks Servo library API for ESP32x's LEDC manager
  // https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h
  bool attached();
  void detach();
  void writeMicroseconds(int value);
  bool _is_attached;
#endif
#ifndef ARDUINO_ARCH_ESP32
  Servo *_servo = nullptr;
#endif
  uint8_t _pin;
  int _max_pulse_width;
  int _min_pulse_width;
  int _frequency;
};
#endif // WS_SERVO_HARDWARE_H