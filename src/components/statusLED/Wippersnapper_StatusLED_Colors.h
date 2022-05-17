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




#define STATUS_LED_KAT_BLINK_TIME                                              \
  120000 ///< How often to blink the status LED while run() executes, if not
         ///< in-use

/** Defines the Wippersnapper status LED states */
typedef enum {
  WS_LED_STATUS_FS_WRITE,
  WS_LED_STATUS_WIFI_CONNECTING,
  WS_LED_STATUS_MQTT_CONNECTING,
  WS_LED_STATUS_WAITING_FOR_REG_MSG,
  WS_LED_STATUS_ERROR_RUNTIME,
  WS_LED_STATUS_KAT,
} ws_led_status_t;
