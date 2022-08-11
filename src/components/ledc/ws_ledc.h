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
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_COMPONENT_LEDC_H
#define WIPPERSNAPPER_COMPONENT_LEDC_H

#include "Wippersnapper.h"

#include "esp32-hal-ledc.h"
#include "esp_err.h"

#define MAX_LEDC_PWMS                                                          \
  16 ///< maximum # of LEDC channels (see: LEDC Chan to Group/Channel/Timer
     ///< Mapping)
#define LEDC_RESOLUTION 10 ///< max LEDC resolution
#define MAX_TIMER_WIDTH 20 ///< max LEDC esp32 timer width

/** Defines a pin attached to an active LEDC timer group */
typedef struct {
  uint8_t pin;   ///< GPIO pin number
  uint8_t chan;  ///< Channel allocated to the pin
  bool isActive; ///< True if the ledcPin's timer has been initialized
} ledcPin_t;

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
class WipperSnapper_Component_LEDC {
public:
  WipperSnapper_Component_LEDC();
  ~WipperSnapper_Component_LEDC();
  uint8_t attachPin(uint8_t pin, double freq);
  void detachPin(uint8_t pin);
  void setDuty(uint8_t pin, uint32_t duty);

private:
  uint8_t _allocateChannel(double freq);
  ledcPin_t _ledcPins[MAX_LEDC_PWMS]; ///< Pool of usable LEDC pins
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_COMPONENT_LEDC_H