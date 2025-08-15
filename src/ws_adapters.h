/*!
 * @file ws_adapters.h
 *
 * This file includes the adapter interfaces at compile-time.
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

#ifndef WS_ADAPTERS_H
#define WS_ADAPTERS_H
/**
 * The following are adapters for use with WiFi modules.
 */
// Adafruit AirLift (ESP32) networking adapter
#if defined(ADAFRUIT_METRO_M4_EXPRESS) ||                                      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT)
#include "adapters/wifi/ws_wifi_airlift.h"
typedef ws_wifi_airlift ws_adapter_wifi;
// ESP8266 networking adapter
#elif defined(ARDUINO_ARCH_ESP8266)
#include "adapters/wifi/ws_wifi_esp8266.h"
typedef ws_wifi_esp8266 ws_adapter_wifi;
#elif defined(ARDUINO_ESP32_DEV) || defined(ESP32_DEV)
#include "adapters/wifi/ws_wifi_esp32.h"
typedef ws_wifi_esp32 ws_adapter_wifi;
// ESP32 networking adapter
#elif defined(ARDUINO_ARCH_ESP32)
#include "adapters/wifi/ws_wifi_esp32.h"
typedef ws_wifi_esp32 ws_adapter_wifi;
// Networking adapters for Raspberry Pi Pico W-series
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W) ||                                  \
    defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#include "adapters/wifi/ws_wifi_pico.h"
typedef ws_wifi_pico ws_adapter_wifi;
// Networking adapter for Arduino Nano 33 IoT and MKR WiFi 1010
#elif defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRWIFI1010)
#include "adapters/wifi/ws_wifi_ninafw.h"
typedef ws_wifi_ninafw ws_adapter_wifi;
/**
 * The following are adapters for use without networking functionality.
 */
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2) ||                                  \
    defined(ARDUINO_RASPBERRY_PI_PICO) ||                                      \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER) ||                      \
    defined(ARDUINO_ADAFRUIT_METRO_RP2350)
#define WS_OFFLINE_ADAPTER
#include "adapters/offline/ws_offline_pico.h"
typedef ws_offline_pico ws_adapter_offline;
#else
#warning "Transport adapter not defined within ws_adapters.h!"
#endif

#ifndef WS_OFFLINE_ADAPTER
#define WS_WIFI_ADAPTER
#endif

#endif // WS_ADAPTERS_H