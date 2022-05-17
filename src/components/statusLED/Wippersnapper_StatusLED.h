/*!
 * @file Wippersnapper_StatusLED.cpp
 *
 * Interfaces for the Wippersnapper status indicator LED/NeoPixel/Dotstar/RGB
 * LED.
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
#ifndef WIPPERSNAPPER_STATUSLED_H
#define WIPPERSNAPPER_STATUSLED_H
#include "Wippersnapper.h"
#include <Adafruit_DotStar.h>
#include <Adafruit_NeoPixel.h>

// Use LEDC for ESP32 arch so we can PWM
#ifdef ARDUINO_ARCH_ESP32
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
// use 12 bit precission for LEDC timer
#define LEDC_TIMER_12_BIT 12
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000
#endif


  // Status LED
  bool statusLEDInit();
  void statusLEDDeinit();
  void setStatusLEDColor(uint32_t color);
  void statusLEDBlink(ws_led_status_t statusState, bool blinkFast = false);
  void statusLEDFade(uint32_t color, int numFades);

#endif // WIPPERSNAPPER_STATUSLED_H