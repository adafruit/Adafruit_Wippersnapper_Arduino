/*!
 * @file Wippersnapper_StatusLED_Colors.h
 *
 * This file contains colors and states for a wippersnapper device's status signal LED
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
#define RED     0xFF0000
#define CYAN    0x00FFFF
#define YELLOW  0xFFFF00
#define GREEN   0x00FF00
#define BLACK   0x000000
#define PINK    0xFF00FF
#define BLUE    0x0000FF

// colors for each status state
#define LED_HW_INIT         PINK
#define LED_NET_CONNECT     CYAN
#define LED_IO_CONNECT      BLUE
#define LED_IO_REGISTER_HW  YELLOW
#define LED_CONNECTED       GREEN
#define LED_ERROR           RED

/** Defines the Wippersnapper status LED states */
typedef enum {
  WS_LED_STATUS_CONNECTED,
  WS_LED_STATUS_ERROR
} ws_led_status_t;
