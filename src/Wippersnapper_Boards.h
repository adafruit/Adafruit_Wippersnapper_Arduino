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
#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
#define BOARD_ID "adafruit-funhouse-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_DOTSTAR
#define STATUS_DOTSTAR_PIN_DATA PIN_DOTSTAR_DATA
#define STATUS_DOTSTAR_PIN_CLK PIN_DOTSTAR_CLOCK
#define STATUS_DOTSTAR_NUM 5
#elif defined(ARDUINO_METRO_ESP32S2)
#define BOARD_ID "adafruit-metro-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 45
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_MAGTAG29_ESP32S2)
#define BOARD_ID "adafruit-magtag-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define NEEDS_STATUS_NEOPIXEL_POWER
#define STATUS_NEOPIXEL_PIN 1
#define STATUS_NEOPIXEL_NUM 4
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
#define BOARD_ID "adafruit-feather-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define NEEDS_STATUS_NEOPIXEL_POWER
#define STATUS_NEOPIXEL_PIN 33
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
#define BOARD_ID "adafruit-feather-esp32s2-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define NEEDS_STATUS_NEOPIXEL_POWER
#define STATUS_NEOPIXEL_PIN 33
#define STATUS_NEOPIXEL_NUM 1
#define PIN_I2C_POWER_INVERTED 7
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2)
#define BOARD_ID "adafruit-qtpy-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define NEEDS_STATUS_NEOPIXEL_POWER
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
#define USB_VID 0xEA60
#define USB_PID 0x10C4
#define BOARD_ID "feather-esp8266"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#define USE_LITTLEFS
#elif defined(ARDUINO_FEATHER_ESP32)
#define BOARD_ID "feather-esp32"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#define USE_LITTLEFS
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
#define BOARD_ID "feather-esp32-v2"
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_LITTLEFS
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
#define BOARD_ID "nano-33-iot"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#define BOARD_ID "mkr-wifi-1010"
#define USE_STATUS_LED
#define STATUS_LED_PIN 6
#else
#warning "Board type not identified within Wippersnapper_Boards.h!"
#endif

#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H