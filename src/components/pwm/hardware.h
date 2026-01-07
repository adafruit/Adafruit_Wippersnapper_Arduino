/*!
 * @file src/components/pwm/hardware.h
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
#ifndef WS_PWM_HARDWARE_H
#define WS_PWM_HARDWARE_H
#include "wippersnapper.h"
#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-ledc.h"
#include "esp_err.h"
#endif

/*!
    @brief  Interface for interacting with hardware's PWM API.
*/
class PWMHardware {
public:
  PWMHardware();
  ~PWMHardware();
  bool AttachPin(uint8_t pin, uint32_t frequency, uint32_t resolution);
  bool DetachPin();
  bool WriteDutyCycle(uint32_t duty);
  uint32_t WriteTone(uint32_t freq);
  uint8_t GetPin();

// Abstractions for LEDC API
#ifdef ARDUINO_ARCH_ESP32
  bool analogWrite(uint32_t value);
#endif
private:
  bool _is_attached;
  uint8_t _pin;
  uint32_t _frequency;
  uint32_t _resolution;
  uint32_t _duty_cycle;
};
#endif // WS_PWM_HARDWARE_H
