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

#if defined(ADAFRUIT_METRO_M4_EXPRESS) ||                                      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT)
#include "network_interfaces/Wippersnapper_AIRLIFT.h"
/** Nina-FW (adafruit fork) networking class */
typedef Wippersnapper_AIRLIFT ws_adapter_wifi;
#elif defined(ARDUINO_ARCH_ESP8266)
#include "network_interfaces/Wippersnapper_ESP8266.h"
/** ESP8266's networking class */
typedef Wippersnapper_ESP8266 ws_adapter_wifi;
#elif defined(ARDUINO_ARCH_ESP32)
#include "network_interfaces/Wippersnapper_ESP32.h"
/** ESP32's networking class */
typedef Wippersnapper_ESP32 ws_adapter_wifi;
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include "network_interfaces/ws_networking_pico.h"
typedef ws_networking_pico ws_adapter_wifi;
#elif defined(RASPBERRY_PI_PICO) || defined(RASPBERRY_PI_PICO_2)
#include "network_interfaces/ws_nonet_pico.h"
typedef ws_nonet_pico ws_adapter_wifi;
#elif defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRWIFI1010)
/** Nina-FW (arduino) networking class */
#include "network_interfaces/Wippersnapper_WIFININA.h"
typedef Wippersnapper_WIFININA ws_adapter_wifi;
#else
#warning "Must define network interface in config.h!"
#endif

#endif // WS_ADAPTERS_H