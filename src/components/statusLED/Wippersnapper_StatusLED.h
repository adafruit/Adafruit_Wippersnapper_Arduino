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
#include <Adafruit_DotStar.h>
#include <Adafruit_NeoPixel.h>

// Use LEDC for ESP32 arch so we can PWM
#ifdef ARDUINO_ARCH_ESP32
#define LEDC_CHANNEL_0                                                         \
  0 ///< use first channel of 16 channels (started from zero)
#define LEDC_TIMER_12_BIT 12 ///< use 12 bit precision for LEDC timer
#define LEDC_BASE_FREQ 5000  ///< use 5000Hz as a LEDC base frequency
#endif

#define STATUS_LED_KAT_BLINK_TIME                                              \
  120000 ///< How often to blink the status LED while run() executes, if not
         ///< in-use

/** Defines the Wippersnapper status LED states */
typedef enum ws_led_status_t {
  WS_LED_STATUS_FS_WRITE,
  WS_LED_STATUS_WIFI_CONNECTING,
  WS_LED_STATUS_MQTT_CONNECTING,
  WS_LED_STATUS_WAITING_FOR_REG_MSG,
  WS_LED_STATUS_ERROR_RUNTIME,
  WS_LED_STATUS_KAT,
} ws_led_status_t;

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
uint32_t ledStatusStateToColor(ws_led_status_t statusState);
void setStatusLEDColor(uint32_t color);
void statusLEDBlink(ws_led_status_t statusState = WS_LED_STATUS_ERROR_RUNTIME);
void statusLEDFade(uint32_t color, int numFades);
void statusLEDSolid(ws_led_status_t statusState);

#endif // WIPPERSNAPPER_STATUSLED_H