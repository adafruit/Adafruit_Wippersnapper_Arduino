/*!
 * @file src/components/pwm/hardware.h
 *
 * Hardware for the pwm.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
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

class ExpanderHardware;

/*!
    @brief  Interface for interacting with hardware's PWM API.
*/
class PWMHardware {
public:
  PWMHardware();
  ~PWMHardware();
  bool attach(uint8_t pin, uint32_t frequency, uint32_t resolution,
              ExpanderHardware *expander_drv);
  bool detach();
  bool write(ws_pwm_Write *msg);
  uint8_t GetPin();
  ExpanderHardware *GetExpanderDriver();

// Abstractions for LEDC API
#ifdef ARDUINO_ARCH_ESP32
  bool analogWrite(uint32_t value);
#endif
private:
  bool writeDutyCycle(uint32_t duty);
  uint32_t writeTone(uint32_t freq);
  bool _is_attached;
  uint8_t _pin;
  uint32_t _frequency;
  uint32_t _resolution;
  uint32_t _duty_cycle;
  ExpanderHardware *_expander_drv = nullptr;
};
#endif // WS_PWM_HARDWARE_H
