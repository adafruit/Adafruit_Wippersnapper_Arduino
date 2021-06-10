/*!
 * @file Wippersnapper_ESP32_nvs.cpp
 *
 * Provisioning helper for the ESP32's non-volatile-storage (NVS)
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
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

  String ssid = nvs.getString("wsNetSSID", "");
  String ssidPass = nvs.getString("wsNetPass", "");
  String aioUser = nvs.getString("wsAIOUser", "");
  String aioPass = nvs.getString("wsAIOKey", "");
  if (ssid == "" || ssidPass == "" || aioUser == "" || aioPass == "") {
    return false;
  }
  // set credentials from NVS
  WS._network_ssid = ssid.c_str();
  WS._network_pass = ssidPass.c_str();
  WS._username = aioUser.c_str();
  WS._key = aioPass.c_str();
  return true;
}

//#endif // USE_NVS