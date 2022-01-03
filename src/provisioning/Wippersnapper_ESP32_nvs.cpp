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
#if defined(ARDUINO_FEATHER_ESP32)
#include "Wippersnapper_ESP32_nvs.h"

/****************************************************************************/
/*!
    @brief    Initializes the ESP32's non-volatile-storage (nvs) at an
                "wsNamespace" namespace location.
*/
/****************************************************************************/
Wippersnapper_ESP32_nvs::Wippersnapper_ESP32_nvs() {
  // Attempt to initialize NVS partition
  if (!nvs.begin("wsNamespace", false)) {
    WS.setStatusLEDColor(RED);
    while (1)
      ;
  }
}

/****************************************************************************/
/*!
    @brief    De-initializes the ESP32's non-volatile-storage (nvs).
*/
/****************************************************************************/
Wippersnapper_ESP32_nvs::~Wippersnapper_ESP32_nvs() { nvs.end(); }

/****************************************************************************/
/*!
    @brief    Reads, validates, and sets credentials from nvs' "wsNamespace"
                namespace.
*/
/****************************************************************************/
void Wippersnapper_ESP32_nvs::parseSecrets() {
  // Parse from
  // https://github.com/adafruit/Adafruit_WebSerial_NVMGenerator/blob/main/wsPartitions.csv
  _ssid = nvs.getString("wsNetSSID", "");
  _ssidPass = nvs.getString("wsNetPass", "");
  _aioUser = nvs.getString("wsAIOUser", "");
  _aioPass = nvs.getString("wsAIOKey", "");

  // Validate getString() calls
  if (_ssid.length() == 0 || _ssidPass.length() == 0 ||
      _aioUser.length() == 0 || _aioPass.length() == 0) {
    WS.setStatusLEDColor(RED);
    while (1)
      ;
  }

  // optional NVS staging url
  _aioURL = nvs.getString("wsAIOURL", "");
  if (_aioURL.length() == 0)
    _aioURL = "io.adafruit.com";

  // Set global configuration strings
  WS._network_ssid = _ssid.c_str();
  WS._network_pass = _ssidPass.c_str();
  WS._username = _aioUser.c_str();
  WS._key = _aioPass.c_str();
  WS._mqttBrokerURL = _aioURL.c_str();
}

#endif // ARDUINO_ARCH_ESP32