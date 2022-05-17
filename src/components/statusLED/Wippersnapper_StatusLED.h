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

// Use LEDC for ESP32 arch so we can PWM
#ifdef ARDUINO_ARCH_ESP32
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
// use 12 bit precission for LEDC timer
#define LEDC_TIMER_12_BIT 12
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000
#endif

#ifdef USE_STATUS_NEOPIXEL
Adafruit_NeoPixel *statusPixel = new Adafruit_NeoPixel(
    STATUS_NEOPIXEL_NUM, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

#ifdef USE_STATUS_DOTSTAR
Adafruit_DotStar *statusPixelDotStar =
    new Adafruit_DotStar(STATUS_DOTSTAR_NUM, STATUS_DOTSTAR_PIN_DATA,
                         STATUS_DOTSTAR_PIN_CLK, DOTSTAR_BRG);
#endif

  // Status LED
  bool statusLEDInit();
  void statusLEDDeinit();
  void setStatusLEDColor(uint32_t color);
  void statusLEDBlink(ws_led_status_t statusState, bool blinkFast = false);
  void statusLEDFade(uint32_t color, int numFades);

#endif // WIPPERSNAPPER_STATUSLED_H