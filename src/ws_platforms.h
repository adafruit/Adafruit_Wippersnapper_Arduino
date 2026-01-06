/*!
 * @file ws_platforms.h
 *
 * This file includes the appropriate networking adapter based on the
 * target platform.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WS_PLATFORMS_H
#define WS_PLATFORMS_H
/**
 * The following are interfaces for supported MCU platforms.
 */
// Espressif ESP8266
#if defined(ARDUINO_ARCH_ESP8266)
#include "platforms/esp8266_wifi.h"
typedef esp8266_wifi ws_adapter_wifi;
// Espressif ESP32 and ESP32x
#elif defined(ARDUINO_ARCH_ESP32)
#include "platforms/esp32_wifi.h"
typedef esp32_wifi ws_adapter_wifi;
// Raspberry Pi Pico W
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W) ||                                  \
    defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#include "platforms/pico_wifi.h"
typedef pico_wifi ws_adapter_wifi;
// Adafruit "AirLift" (ESP32 coprocessor)
#elif defined(ADAFRUIT_METRO_M4_EXPRESS) ||                                      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT)
#include "platforms/airlift_wifi.h"
typedef airlift_wifi ws_adapter_wifi;
// Arduino Nina-Firmware (ESP32 Coprocessor)
#elif defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRWIFI1010)
#include "platforms/ninafw_wifi.h"
typedef ninafw_wifi ws_adapter_wifi;
// RP2040/RP2350 without WiFi (e.g., Adafruit Feather RP2040, Raspberry Pi Pico)
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2) ||                                  \
    defined(ARDUINO_RASPBERRY_PI_PICO) ||                                      \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER) ||                      \
    defined(ARDUINO_ADAFRUIT_METRO_RP2350)
#include "platforms/pico_offline.h"
typedef pico_offline ws_adapter_offline;
#else
#warning "Platform not defined within ws_platforms.h!"
#endif

#endif // WS_PLATFORMS_H