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
#ifndef WS_LEDC_H
#define WS_LEDC_H

#include "Wippersnapper.h"

#include "esp32-hal-ledc.h"
#include "esp_err.h"

#define LEDC_CH_ERR 255 ///< LEDC channel error
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
class ws_ledc {
public:
  ws_ledc();
  ~ws_ledc();
  uint8_t attachPin(uint8_t pin, double freq, uint8_t resolution);
  void detachPin(uint8_t pin);

  void setDuty(uint8_t pin, uint32_t duty);
  void analogWrite(uint8_t pin, int value);
  // void tone(uint8_t pin, uint32_t freq);

private:
  // TODO uScore
  uint8_t allocateChannel(double freq, uint8_t resolution);
  uint8_t getChannel(uint8_t pin);
  uint8_t getInactiveChannel();
  ledcPin_t _ledcPins[MAX_LEDC_PWMS]; ///< Pool of usable LEDC pins
};
extern Wippersnapper WS;

#endif // ws_ledc_H