/*!
 * @file Wippersnapper_ESP32_nvs.h
 *
 * Provisioning helper for the ESP32's non-volatile-storage (NVS)
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
#ifndef WIPPERSNAPPER_ESP32_NVS_H
#define WIPPERSNAPPER_ESP32_NVS_H

#include "Wippersnapper.h"
#include <Preferences.h>

class Wippersnapper;
class Wippersnapper_ESP32_nvs {
public:
  Wippersnapper_ESP32_nvs();
  ~Wippersnapper_ESP32_nvs();

  bool validateNVSConfig();
  bool setNVSConfig();

  Preferences nvs;

private:
  String _ssid;
  String _ssidPass;
  String _aioUser;
  String _aioPass;
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_ESP32_NVS_H