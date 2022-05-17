/*!
 * @file Wippersnapper_StatusLED.h
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

#define RED 0xFF0000    ///< Red (as a uint32)
#define CYAN 0x00FFFF   ///< Cyan (as a uint32)
#define YELLOW 0xFFFF00 ///< Yellow (as a uint32)
#define GREEN 0x00A300  ///< Green (as a uint32)
#define BLACK 0x000000  ///< Black (as a uint32)
#define PINK 0xFF00FF   ///< Pink (as a uint32)
#define BLUE 0x0000FF   ///< Blue (as a uint32)
#define AMBER 0xFFBF00  ///< Amber (as a uint32)

// colors for each status state
#define LED_NET_CONNECT PINK      ///< Network connection state
#define LED_IO_CONNECT BLUE       ///< MQTT broker connection state
#define LED_IO_REGISTER_HW YELLOW ///< Hardware registration state
#define LED_CONNECTED GREEN       ///< Successful registration state
#define LED_ERROR RED             ///< Error state

// Status LED
bool statusLEDInit();
void statusLEDDeinit();
void setStatusLEDColor(uint32_t color);
void statusLEDBlink(ws_led_status_t statusState = WS_LED_STATUS_ERROR_RUNTIME,
                    bool blinkFast = false);
void statusLEDFade(uint32_t color, int numFades);

#endif // WIPPERSNAPPER_STATUSLED_H