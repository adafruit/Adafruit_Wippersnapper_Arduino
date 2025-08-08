/*!
 * @file Wippersnapper_Boards.h
 *
 * This file determines hardware/board type at compile-time.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef ADAFRUIT_WIPPERSNAPPER_BOARDS_H
#define ADAFRUIT_WIPPERSNAPPER_BOARDS_H

#if defined(ADAFRUIT_PYPORTAL)
#define BOARD_ID "pyportal-tinyusb"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 2
#define STATUS_NEOPIXEL_NUM 1
#define SD_CS_PIN 10
#elif defined(ADAFRUIT_PYPORTAL_M4_TITANO)
#define BOARD_ID "pyportal-titano-tinyusb"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 2
#define STATUS_NEOPIXEL_NUM 1
#define SD_CS_PIN 10
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
#define BOARD_ID "metro-m4-airliftlite-tinyusb"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 40
#define STATUS_NEOPIXEL_NUM 1
#define SD_CS_PIN 10
#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
#define BOARD_ID "funhouse"                      ///< Board ID
#define USE_TINYUSB                              ///< Enable TinyUSB
#define USE_STATUS_DOTSTAR                       ///< Enable DotStar
#define USE_DISPLAY                              ///< Enable Display
#define STATUS_DOTSTAR_PIN_DATA PIN_DOTSTAR_DATA ///< DotStar Data Pin
#define STATUS_DOTSTAR_PIN_CLK PIN_DOTSTAR_CLOCK ///< DotStar Clock Pin
#define STATUS_DOTSTAR_NUM 5                     ///< Number of DotStar LEDs
#define STATUS_DOTSTAR_COLOR_ORDER DOTSTAR_GBR   ///< DotStar Color Order
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 10
#elif defined(ARDUINO_METRO_ESP32S2)
#define BOARD_ID "metroesp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 45
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 10
#elif defined(ARDUINO_METRO_ESP32S3)
#define BOARD_ID "metroesp32s3"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 46
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 33
#elif defined(ARDUINO_MAGTAG29_ESP32S2)
#define BOARD_ID "magtag"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 1
#define STATUS_NEOPIXEL_NUM 4
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 10
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
#define BOARD_ID "feather-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 33
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 10
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
#define BOARD_ID "feather-esp32s2-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 33
#define STATUS_NEOPIXEL_NUM 1
#define PIN_I2C_POWER_INVERTED 7
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 10
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT)
#define BOARD_ID "feather-esp32s2-reverse-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 10
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
#define BOARD_ID "feather-esp32s3"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
#define BOARD_ID "feather-esp32s3-4mbflash-2mbpsram"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
#define BOARD_ID "feather-esp32s3-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT)
#define BOARD_ID "feather-esp32s3-reverse-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2)
#define BOARD_ID "qtpy-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define I2c_STEMMA_WIRE1
#define SD_CS_PIN 10
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM)
#define BOARD_ID "qtpy-esp32s3"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define I2c_STEMMA_WIRE1
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2)
#define BOARD_ID "qtpy-esp32s3-n4r2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define I2c_STEMMA_WIRE1
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
#define BOARD_ID "qtpy-esp32c3"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define SD_CS_PIN 33
#elif defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
#define BOARD_ID "feather-esp8266"
#define USE_LITTLEFS
#define USE_STATUS_LED
#define STATUS_LED_PIN 0
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_ITSYBITSY_ESP32)
#define BOARD_ID "itsybitsy-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 33
#elif defined(ARDUINO_FEATHER_ESP32)
#define BOARD_ID "feather-esp32"
#define USE_LITTLEFS
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#define SD_CS_PIN 33
#elif defined(ARDUINO_ESP32_DEV) || defined(ESP32_DEV)
#define BOARD_ID "feather-esp32"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
#define BOARD_ID "feather-esp32-v2"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define SD_CS_PIN 33
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
#define BOARD_ID "qtpy-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define I2c_STEMMA_WIRE1
#define SD_CS_PIN 33
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
#define BOARD_ID "nano-33-iot"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#define SD_CS_PIN 33
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#define BOARD_ID "mkrwifi1010"
#define USE_STATUS_LED
#define STATUS_LED_PIN 6
#define SD_CS_PIN 33
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#define BOARD_ID "rpi-pico-w"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN 64
#define SD_CS_PIN 17
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#define BOARD_ID "rpi-pico-2w"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN 64
#define SD_CS_PIN 17
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define BOARD_ID "rpi-pico"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#define SD_CS_PIN 17
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2)
#define BOARD_ID "rpi-pico-2"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#define SD_CS_PIN 17
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
#define BOARD_ID "feather-rp2040-adalogger"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define SD_USE_SPI_1
#define SD_CS_PIN 23
#elif defined(ARDUINO_ADAFRUIT_METRO_RP2350)
#define BOARD_ID "metro-rp2350"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NUM_NEOPIXEL
#define SD_USE_SPI_1
#define SD_CS_PIN 39
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6)
#define BOARD_ID "feather-esp32c6"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_ESP32S3_DEV)
#define BOARD_ID "esp32s3-devkitc-1-n8"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 48
#define STATUS_NEOPIXEL_NUM 1
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
// Update board ID if PSRAM is present, needs new board definition
#endif
#elif defined(ARDUINO_XIAO_ESP32S3)
#define BOARD_ID "xiao-esp32s3"
#define BOARD_HAS_PSRAM
#define USE_PSRAM
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
// XIAO S3 Sense Camera addon SD card CS pin GPIO21, or D2/GPIO3 on grove breakout
#define SD_CS_PIN 21 
#else
#warning "Board type not identified within Wippersnapper_Boards.h!"
#endif

#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H
