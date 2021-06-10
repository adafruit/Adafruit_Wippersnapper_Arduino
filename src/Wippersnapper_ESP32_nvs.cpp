/*!
 * @file Wippersnapper_StatusLED.cpp
 *
 * Interfaces for the Wippersnapper status indicator LED/NeoPixel/Dotstar/RGB
 * LED.
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
//#if defined(USE_NVS)
#include "Wippersnapper_ESP32_nvs.h"

// ctor
Wippersnapper_ESP32_nvs::Wippersnapper_ESP32_nvs() {
  // init. nvs, read-only
  nvs.begin("wsNamespace", false);
}

// dtor
Wippersnapper_ESP32_nvs::~Wippersnapper_ESP32_nvs() { nvs.end(); }

bool Wippersnapper_ESP32_nvs::validateNVS() {
    // TODO: convert these to char., save for reuse
    // so we dont need to re-access
    String ssid = nvs.getString("wsNetSSID", "");
    String ssidPass = nvs.getString("wsNetPass", "");
    String aioUser = nvs.getString("wsAIOUser", "");
    String aioPass = nvs.getString("wsAIOKey", "");
    if (ssid == "" || ssidPass == "" || \
        aioUser == "" || aioPass == "") {
            return false;
    }
    return true;
}

//#endif // USE_NVS