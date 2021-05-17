/*!
 * @file Wippersnapper_Boards.h
 *
 * This file determines board type and includes network
 * interface code.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
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
    // Use TinyUSB - for SAMD/NRF52 Cores ONLY!
    #define USE_TINYUSB
    // Status Indicator
    #define STATUS_NEOPIXEL
    #define STATUS_NEOPIXEL_PIN  2
    #define STATUS_NEOPIXEL_LENGTH 1
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
    #define USB_VID 0x239A
    #define USB_PID 0x8038
    #define BOARD_ID "adafruit-metro-m4-airliftlite"
    // Use TinyUSB - for SAMD/NRF52 Cores ONLY!
    #define USE_TINYUSB
    // Status Indicator
    #define STATUS_NEOPIXEL
    #define STATUS_NEOPIXEL_PIN  40
    #define STATUS_NEOPIXEL_LENGTH 1
#elif defined(ADAFRUIT_METRO_M4_EXPRESS)
    #define USB_VID 0x239A
    #define USB_PID 0x8021
    #define BOARD_ID "adafruit-metro-m4"
    // Use TinyUSB - for SAMD/NRF52 Cores ONLY!
    #define USE_TINYUSB
    // Status Indicator
    #define STATUS_NEOPIXEL
    #define STATUS_NEOPIXEL_PIN  40
    #define STATUS_NEOPIXEL_LENGTH 1
#elif defined(ARDUINO_ARCH_ESP8266)
    // Feather Huzzah ESP8266, CP2104 USB/UART Bridge Controller
    #define USB_VID 0xEA60
    #define USB_PID 0x10C4
    #define BOARD_ID "adafruit-huzzah-8266"
    // Status Indicator
    #define STATUS_LED
    #define STATUS_LED_PIN  13
#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
        #define BOARD_ID "adafruit-funhouse-esp32s2"
        // TODO: replace status LED with dotstar!
        #define STATUS_LED
        #define STATUS_LED_PIN  LED_BUILTIN
#elif defined(ARDUINO_ARCH_ESP32)
        #define BOARD_ID "esp32"
        // TODO: replace status LED with dotstar!
        #define STATUS_LED
        #define STATUS_LED_PIN  LED_BUILTIN
#else
  #warning "Board not identified within Wippersnapper_Boards.h!"
#endif


#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H