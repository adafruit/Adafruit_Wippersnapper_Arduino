/*!
 * @file ws_boards.h
 *
 * This file contains all board-specific definitions for Adafruit Wippersnapper.
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

#ifndef WS_BOARDS_H
#define WS_BOARDS_H

#if defined(ADAFRUIT_PYPORTAL)
#define BOARD_ID "pyportal-tinyusb"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 2
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ADAFRUIT_PYPORTAL_M4_TITANO)
#define BOARD_ID "pyportal-titano-tinyusb"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 2
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
#define BOARD_ID "metro-m4-airliftlite-tinyusb"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 40
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
#define BOARD_ID "funhouse"                      ///< Board ID
#define USE_TINYUSB                              ///< Enable TinyUSB
#define USE_STATUS_DOTSTAR                       ///< Enable DotStar
#define STATUS_DOTSTAR_PIN_DATA PIN_DOTSTAR_DATA ///< DotStar Data Pin
#define STATUS_DOTSTAR_PIN_CLK PIN_DOTSTAR_CLOCK ///< DotStar Clock Pin
#define STATUS_DOTSTAR_NUM 5                     ///< Number of DotStar LEDs
#define STATUS_DOTSTAR_COLOR_ORDER DOTSTAR_GBR   ///< DotStar Color Order
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#elif defined(ARDUINO_METRO_ESP32S2)
#define BOARD_ID "metroesp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 45
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#elif defined(ARDUINO_ESP32S3_DEV)
#define BOARD_ID "esp32s3-devkitc-1-n8"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 48
#define STATUS_NEOPIXEL_NUM 1
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#endif
#elif defined(ARDUINO_METRO_ESP32S3)
#define BOARD_ID "metroesp32s3"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 46
#define STATUS_NEOPIXEL_NUM 1
#define BOOT_BUTTON PIN_BUTTON1 ///< Normalized boot button pin
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#elif defined(ARDUINO_MAGTAG29_ESP32S2)
#define BOARD_ID "magtag"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 1
#define STATUS_NEOPIXEL_NUM 4
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
#define BOARD_ID "feather-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 33
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
#define BOARD_ID "feather-esp32s2-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN 33
#define STATUS_NEOPIXEL_NUM 1
#define PIN_I2C_POWER_INVERTED 7
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT)
#define BOARD_ID "feather-esp32s2-reverse-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
#define BOARD_ID "feather-esp32s3"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
#define BOARD_ID "feather-esp32s3-4mbflash-2mbpsram"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
#define BOARD_ID "feather-esp32s3-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT)
#define BOARD_ID "feather-esp32s3-reverse-tft"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NEOPIXEL_NUM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2)
#define BOARD_ID "qtpy-esp32s2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define I2C_STEMMA_WIRE1
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM)
#define BOARD_ID "qtpy-esp32s3"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define I2C_STEMMA_WIRE1
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2)
#define BOARD_ID "qtpy-esp32s3-n4r2"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define I2C_STEMMA_WIRE1
#define BOOT_BUTTON 0
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
#define BOARD_ID "qtpy-esp32c3"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define BOOT_BUTTON 0
#elif defined(WS_DFROBOT_BEETLE_ESP32C3)
// Note: this board reuses a generic preprocessor define
// espressif/arduino-esp32@fcd4799c6de6eb5a5a8eba94818adf770238ecc0
// rather than creating one unique to the device.
#define BOARD_ID "dfrobot-beetle-esp32c3"
#define USE_LITTLEFS
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#elif defined(ARDUINO_ESP32C3_DEV)
#define BOARD_ID "dfrobot-beetle-esp32c3"
#define USE_LITTLEFS
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#elif defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
#define BOARD_ID "feather-esp8266"
#define USE_LITTLEFS
#define USE_STATUS_LED
#define STATUS_LED_PIN 0
#elif defined(ARDUINO_ADAFRUIT_ITSYBITSY_ESP32)
#define BOARD_ID "itsybitsy-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#elif defined(ARDUINO_FEATHER_ESP32)
#define BOARD_ID "feather-esp32"
#define USE_LITTLEFS
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#elif defined(ARDUINO_SPARKLEMOTION_ESP32)
#define BOARD_ID "sparklemotion-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_SPARKLEMOTIONMINI_ESP32)
#define BOARD_ID "sparklemotionmini-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_SPARKLEMOTIONSTICK_ESP32)
#define BOARD_ID "sparklemotionstick-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_ESP32_DEV) || defined(ESP32_DEV)
#define BOARD_ID "feather-esp32"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
#define BOARD_ID "feather-esp32-v2"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON BUTTON
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
#define BOARD_ID "qtpy-esp32"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#define I2C_STEMMA_WIRE1
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
#define BOARD_ID "nano-33-iot"
#define USE_STATUS_LED
#define STATUS_LED_PIN 13
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#define BOARD_ID "mkrwifi1010"
#define USE_STATUS_LED
#define STATUS_LED_PIN 6
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#define BOARD_ID "rpi-pico-w"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN 64
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#define BOARD_ID "rpi-pico-2w"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN 64
#elif defined(ARDUINO_XIAO_ESP32S3)
#define BOARD_ID "xiao-esp32s3"
#define BOARD_HAS_PSRAM
#define USE_PSRAM
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define BOARD_ID "rpi-pico"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2)
#define BOARD_ID "rpi-pico-2"
#define USE_TINYUSB
#define USE_STATUS_LED
#define STATUS_LED_PIN LED_BUILTIN
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
#define BOARD_ID "feather-rp2040-adalogger"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define SD_USE_SPI_1
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
#define BOARD_ID "feather-rp2040"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#define SD_USE_SPI_1
#elif defined(ARDUINO_ADAFRUIT_METRO_RP2350)
#define BOARD_ID "metro-rp2350"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM NUM_NEOPIXEL
#define SD_USE_SPI_1
#elif defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)
#define BOARD_ID "fruitjam"
#define USE_TINYUSB
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 5
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6)
#define BOARD_ID "feather-esp32c6"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_NEOPIXEL
#define STATUS_NEOPIXEL_NUM 1
#elif defined(ARDUINO_DFROBOT_FIREBEETLE_2_ESP32C5)
#define BOARD_ID "dfrobot-firebeetle2-esp32c5" ///< Board ID
#define USE_LITTLEFS               ///< Use LittleFS for provisioning
#define USE_STATUS_LED             ///< Single-colour onboard LED
#define STATUS_LED_PIN LED_BUILTIN ///< GPIO15/D13, shared with GDI LCD_BL
#define BOOT_BUTTON 28             ///< BOOT button (GPIO28)
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#endif
#elif defined(ARDUINO_ADAFRUIT_QUALIA_S3_RGB666)
#define BOARD_ID "qualia-s3-rgb666"
#define USE_TINYUSB
#define USE_PSRAM
#define BOOT_BUTTON 0
#elif defined(ARDUINO_LILYGO_T_DISPLAY_S3)
#define BOARD_ID "lilygo-t-display-s3"
#define USE_TINYUSB
#define USE_PSRAM     ///< Board has PSRAM, use it for dynamic memory allocation
#define BOOT_BUTTON 0 ///< BUTTON_1 (GPIO0) is the boot button
// The on-board ST7789 (170x320, 8-bit i8080 parallel) is fully described by
// the ws.display.Add protobuf: data/cs/dc/rst/write/read via
// I8080PinDescriptor, backlight via Add.backlight (BacklightConfig), and the
// panel power-enable rail via Add.power (a prerequisite ws.digital.Add) —
// no board-hardcoded display pins here.
#elif defined(WS_LILYGO_T_DONGLE_C5)
#define BOARD_ID "lilygo-t-dongle-c5" ///< Board ID
#define USE_LITTLEFS                  ///< Use LittleFS for provisioning
#define USE_STATUS_DOTSTAR            ///< Onboard RGB LED is an APA102/DotStar
#define STATUS_DOTSTAR_PIN_DATA 5     ///< DotStar Data Pin (LED_DI)
#define STATUS_DOTSTAR_PIN_CLK 4      ///< DotStar Clock Pin (LED_CI)
#define STATUS_DOTSTAR_NUM 1          ///< Number of DotStar LEDs
#define STATUS_DOTSTAR_COLOR_ORDER DOTSTAR_BGR ///< DotStar Color Order
#define BOOT_BUTTON 28                         ///< Normalized boot button pin
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#endif
#elif defined(WS_ESPRESSIF_ESP32C5_DEVKITC_1_N8R4) ||                          \
    defined(ARDUINO_ESP32C5_DEV)
// Fallback: boards without a boards.txt entry of their own (e.g. the T-Dongle
// C5) build with the generic Espressif esp32c5 board, so ARDUINO_ESP32C5_DEV is
// not unique. They get a WS_ define from boards.local.txt (Arduino) or
// build_flags (PlatformIO); without one, assume Espressif's own devkit.
#define BOARD_ID "esp32c5-devkitc-1-n8r4"
#define USE_LITTLEFS
#define USE_STATUS_NEOPIXEL
#define STATUS_NEOPIXEL_PIN PIN_RGB_LED
// PIN_RGB_LED = 27, or GPIO_NUM+27 if using RGBwrite()
#define STATUS_NEOPIXEL_NUM 1
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#endif
#elif defined(WS_ESP32S31_COREBOARD_1)
// Builds with the generic Espressif esp32s31 board (ARDUINO_ESP32S31_DEV), so
// WS_ESP32S31_COREBOARD_1 comes from boards.local.txt (Arduino) or
// build_flags (PlatformIO). Ethernet pins come from the BSP's esp32s31 variant.
#define BOARD_ID "esp32s31-coreboard-1" ///< Board ID
#define USE_LITTLEFS                    ///< Use LittleFS for provisioning
#define USE_ETHERNET                    ///< Onboard YT8531DC RGMII Ethernet PHY
#define USE_STATUS_NEOPIXEL             ///< Onboard WS2812 RGB LED
#define STATUS_NEOPIXEL_PIN 60          ///< RGB LED data (GPIO60)
#define STATUS_NEOPIXEL_NUM 1           ///< Number of NeoPixels
#define BOOT_BUTTON 61                  ///< BOOT button (GPIO61)
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#endif
#elif defined(WS_ESP32S31_KORVO_1)
// Builds with the generic Espressif esp32s31 board (ARDUINO_ESP32S31_DEV), so
// WS_ESP32S31_KORVO_1 comes from boards.local.txt (Arduino) or
// build_flags (PlatformIO). The generic variant's RGB LED, I2C and Ethernet
// pins are the Function-CoreBoard-1's, not this board's.
#define BOARD_ID "esp32s31-korvo-1" ///< Board ID
#define USE_LITTLEFS                ///< Use LittleFS for provisioning
#define USE_STATUS_NEOPIXEL         ///< Onboard WS2812B RGB LED
#define STATUS_NEOPIXEL_PIN 37      ///< RGB LED data (WS2812_CTRL, GPIO37)
#define STATUS_NEOPIXEL_NUM 1       ///< Number of NeoPixels
#define BOOT_BUTTON 61              ///< BOOT button (GPIO61, also LCD_SCK)
// 4.3" RGB565 dotclock LCD sub-board (ESP32-S3-LCD-EV-Board-SUB3). Named like
// the Qualia variant's TFT_* pins: R1-R5/G0-G5/B1-B5 are the 5-6-5 data lines
// LSB first (DB11-15, DB5-10, DB0-4).
#define TFT_DE 43                   ///< LCD_H_EN
#define TFT_VSYNC 45                ///< LCD_V_SYNC
#define TFT_HSYNC 44                ///< LCD_H_SYNC
#define TFT_PCLK 40                 ///< LCD_PCLK
#define TFT_DISP 38 ///< LCD_DISP display enable (also LCD_CS, Boot Mode 0)
#define TFT_R1 19   ///< DB11 (R3)
#define TFT_R2 33   ///< DB12 (R4)
#define TFT_R3 34   ///< DB13 (R5)
#define TFT_R4 35   ///< DB14 (R6)
#define TFT_R5 36   ///< DB15 (R7)
#define TFT_G0 13   ///< DB5 (G2)
#define TFT_G1 14   ///< DB6 (G3)
#define TFT_G2 15   ///< DB7 (G4)
#define TFT_G3 16   ///< DB8 (G5)
#define TFT_G4 17   ///< DB9 (G6)
#define TFT_G5 18   ///< DB10 (G7)
#define TFT_B1 8    ///< DB0 (B3)
#define TFT_B2 9    ///< DB1 (B4)
#define TFT_B3 10   ///< DB2 (B5)
#define TFT_B4 11   ///< DB3 (B6)
#define TFT_B5 12   ///< DB4 (B7)
#ifdef BOARD_HAS_PSRAM
#define USE_PSRAM ///< Board has PSRAM, use it for dynamic memory allocation
#endif
#else
#warning "Board type not identified within ws_boards.h!"
#endif

#endif // WS_BOARDS_H
