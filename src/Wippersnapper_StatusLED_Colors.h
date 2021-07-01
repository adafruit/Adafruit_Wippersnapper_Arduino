/*!
 * @file Wippersnapper_StatusLED_Colors.h
 *
 * This file contains colors and states for a wippersnapper device's status
 * signal LED
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#define RED 0xFF0000    ///< Red (as a uint32)
#define CYAN 0x00FFFF   ///< Cyan (as a uint32)
#define YELLOW 0xFFFF00 ///< Yellow (as a uint32)
#define GREEN 0x00A300  ///< Green (as a uint32)
#define BLACK 0x000000  ///< Black (as a uint32)
#define PINK 0xFF00FF   ///< Pink (as a uint32)
#define BLUE 0x0000FF   ///< Blue (as a uint32)

// colors for each status state
#define LED_HW_INIT PINK          ///< Initialization state
#define LED_NET_CONNECT CYAN      ///< Network connection state
#define LED_IO_CONNECT BLUE       ///< MQTT broker connection state
#define LED_IO_REGISTER_HW YELLOW ///< Hardware registration state
#define LED_CONNECTED GREEN       ///< Successful registration state
#define LED_ERROR RED             ///< Error state

#define STATUS_LED_KAT_BLINK_TIME                                              \
  120000 ///< How often to blink the status LED while run() executes, if not
         ///< in-use

/** Defines the Wippersnapper status LED states */
typedef enum {
  WS_LED_STATUS_FS_WRITE,
  WS_LED_STATUS_CONNECTED,
  WS_LED_STATUS_KAT,
  WS_LED_STATUS_ERROR
} ws_led_status_t;
