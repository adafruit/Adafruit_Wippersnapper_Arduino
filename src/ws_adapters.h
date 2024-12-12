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
#include "adapters/network_interfaces/Wippersnapper_AIRLIFT.h"
typedef Wippersnapper_AIRLIFT ws_adapter_wifi;
// ESP8266 networking adapter
#elif defined(ARDUINO_ARCH_ESP8266)
#include "adapters/network_interfaces/Wippersnapper_ESP8266.h"
typedef Wippersnapper_ESP8266 ws_adapter_wifi;
// ESP32 networking adapter
#elif defined(ARDUINO_ARCH_ESP32)
#include "adapters/network_interfaces/Wippersnapper_ESP32.h"
typedef Wippersnapper_ESP32 ws_adapter_wifi;
// Networking adapters for Raspberry Pi Pico W-series
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include "adapters/network_interfaces/ws_networking_pico.h"
typedef ws_networking_pico ws_adapter_wifi;
// Networking adapter for Arduino Nano 33 IoT and MKR WiFi 1010
#elif defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRWIFI1010)
#include "adapters/network_interfaces/Wippersnapper_WIFININA.h"
typedef Wippersnapper_WIFININA ws_adapter_wifi;
/**
 * The following are adapters for use with non-networked devices.
*/
#elif defined(RASPBERRY_PI_PICO) || defined(RASPBERRY_PI_PICO_2)
#include "adapters/network_interfaces/ws_nonet_pico.h"
typedef ws_nonet_pico ws_adapter_offline;
#else
#warning "Must define network interface in config.h!"
#endif

#endif // WS_ADAPTERS_H