/*!
 * @file Wippersnapper_Networking.h
 *
 * This file includes network interfaces at compile-time.
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

#ifndef WIPPERSNAPPER_NETWORKING_H
#define WIPPERSNAPPER_NETWORKING_H

#ifndef WL_MAC_ADDR_LENGTH
#define WL_MAC_ADDR_LENGTH 6 ///< MAC address length - from RP2040 BSP
#endif
#define WS_MAX_ALT_WIFI_NETWORKS 3 ///< Maximum number of alternative networks

#if defined(ADAFRUIT_METRO_M4_EXPRESS) ||                                      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT) ||            \
    defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)
#include "network_interfaces/Wippersnapper_AIRLIFT.h"
/** Nina-FW (adafruit fork) networking class */
typedef Wippersnapper_AIRLIFT Wippersnapper_WiFi;
#elif defined(ARDUINO_ARCH_ESP8266)
#include "network_interfaces/Wippersnapper_ESP8266.h"
/** ESP8266's networking class */
typedef Wippersnapper_ESP8266 Wippersnapper_WiFi;
#elif defined(ARDUINO_ARCH_ESP32)
#include "network_interfaces/Wippersnapper_ESP32.h"
/** ESP32's networking class */
typedef Wippersnapper_ESP32 Wippersnapper_WiFi;
#elif defined(ARDUINO_ARCH_RP2040)
#include "network_interfaces/ws_networking_pico.h"
typedef ws_networking_pico Wippersnapper_WiFi;
#else
#warning "Must define network interface in config.h!"
#endif

#endif // WIPPERSNAPPER_NETWORKING_H