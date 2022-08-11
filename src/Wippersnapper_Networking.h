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

#if defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_METRO_M4_EXPRESS) || defined(USE_AIRLIFT)
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
#elif defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRWIFI1010)
/** Nina-FW (arduino) networking class */
#include "network_interfaces/Wippersnapper_WIFININA.h"
typedef Wippersnapper_WIFININA Wippersnapper_WiFi;
#else
#warning "Must define network interface in config.h!"
#endif

#endif // WIPPERSNAPPER_NETWORKING_H