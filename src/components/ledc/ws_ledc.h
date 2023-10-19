/*!
 * @file ws_ledc.h
 *
 * High-level interface for ESP32's LED Control (LEDC) peripheral,
 * to be used by PWM and Servo drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022-2023 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_LEDC_H
#define WS_LEDC_H

#include "Wippersnapper.h"

#include "esp32-hal-ledc.h"
#include "esp_err.h"

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  High-level interface for the ESP32/ESP32-Sx/ESP32-Cx LED
            Control (LEDC) peripheral. Instead of specifying a timer or
            channel, this class automatically allocates a channel and
            associates it with a pin. Underlying esp32-hal-ledc performs
            timer management and handles the low-level LEDC peripheral API
            calls.
*/
/**************************************************************************/
class ws_ledc {
public:
  ws_ledc();
  ~ws_ledc();
  bool attachPin(uint8_t pin, uint32_t freq, uint8_t resolution);
  bool detachPin(uint8_t pin);
  // LEDC-API
  bool setDuty(uint8_t pin, uint32_t duty);
  bool analogWrite(uint8_t pin, int value);
  uint32_t tone(uint8_t pin, uint32_t freq);
};
extern Wippersnapper WS;

#endif // ws_ledc_H