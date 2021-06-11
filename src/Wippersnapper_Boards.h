/*!
 * @file Wippersnapper_Boards.h
 *
 * This file determines hardware/board type at compile-time.
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

#ifndef ADAFRUIT_WIPPERSNAPPER_BOARDS_H
#define ADAFRUIT_WIPPERSNAPPER_BOARDS_H

#if defined(ADAFRUIT_PYPORTAL)
#define USB_VID 0x239A
#define USB_PID 0x8036
#define BOARD_ID "adafruit-pyportal-m4"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 2
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
#define USB_VID 0x239A
#define USB_PID 0x8038
#define BOARD_ID "adafruit-metro-m4-airliftlite"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 40
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_ARCH_ESP8266)
// Feather Huzzah ESP8266, CP2104 USB/UART Bridge Controller
#define USB_VID 0xEA60
#define USB_PID 0x10C4
#define BOARD_ID "adafruit-huzzah-8266"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
#define BOARD_ID "adafruit-funhouse-esp32s2"
#define USE_STATUS_DOTSTAR
#define STATUS_DOTSTAR_PIN_DATA PIN_DOTSTAR_DATA
#define STATUS_DOTSTAR_PIN_CLK PIN_DOTSTAR_CLOCK
#define STATUS_DOTSTAR_NUM 5
#elif defined(ARDUINO_ARCH_ESP32)
#define BOARD_ID "adafruit-huzzah-32"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#define USE_NVS
#else
#warning "Board type not identified within Wippersnapper_Boards.h!"
#endif

#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H