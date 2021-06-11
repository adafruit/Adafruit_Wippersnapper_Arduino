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

bool Wippersnapper_ESP32_nvs::validateNVSConfig() {
  _ssid = nvs.getString("wsNetSSID", "");
  _ssidPass = nvs.getString("wsNetPass", "");
  _aioUser = nvs.getString("wsAIOUser", "");
  _aioPass = nvs.getString("wsAIOKey", "");
  // validate config properly set in partition
  if (_ssid == "" || _ssidPass == "" || _aioUser == "" || _aioPass == "") {
    return false;
  }
  return true;
}

bool Wippersnapper_ESP32_nvs::setNVSConfig() {
  WS._network_ssid = _ssid.c_str();
  WS._network_pass = _ssidPass.c_str();
  WS._username = _aioUser.c_str();
  WS._key = _aioPass.c_str();
  WS_DEBUG_PRINT("SSID: ");WS_DEBUG_PRINTLN(WS._network_ssid);
  WS_DEBUG_PRINT("SSIDPASS: ");WS_DEBUG_PRINTLN(WS._network_pass);
  WS_DEBUG_PRINT("WS._username: ");WS_DEBUG_PRINTLN(WS._username);
  WS_DEBUG_PRINT("WS._key: ");WS_DEBUG_PRINTLN(WS._key);
}

//#endif // USE_NVS