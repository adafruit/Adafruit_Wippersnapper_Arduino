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
#include "Wippersnapper_V2.h"

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-ledc.h"
#include "esp_err.h"

#define LEDC_TIMER_WIDTH                                                       \
  12 ///< timer width to request from LEDC manager component, in bits (NOTE:
     ///< While ESP32x can go up to 16 bit timer width, ESP32-S2 does not work
     ///< at this resolution. So, for the purposes of keeping this library
     ///< compatible with multiple ESP32x platforms, the timer width has been
     ///< scaled down to 10 bits and the calculation adjusted accordingly)
#else
#include <Servo.h>
#endif

#define MIN_SERVO_PULSE_WIDTH 500 ///< Default min. servo pulse width of 500uS

/**************************************************************************/
/*!
    @brief  Interface for interacting with hardware's Servo API.
*/
/**************************************************************************/
class ServoHardware {
public:
  ServoHardware(int pin, int min_pulse_width, int max_pulse_width,
                int frequency);
  ~ServoHardware();
  bool ServoAttach();
  void ServoWrite(int value);
  uint8_t GetPin();

private:
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