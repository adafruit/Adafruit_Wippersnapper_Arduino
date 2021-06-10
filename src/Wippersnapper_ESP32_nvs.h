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
#ifndef WIPPERSNAPPER_ESP32_NVS_H
#define WIPPERSNAPPER_ESP32_NVS_H

#include "Wippersnapper.h"
#include <Preferences.h>

class Wippersnapper;
class Wippersnapper_ESP32_nvs {
public:
  Wippersnapper_ESP32_nvs();
  ~Wippersnapper_ESP32_nvs();

  bool validateNVS();
  //void setNVS();

  const char *io_username = NULL;
  const char *io_key = NULL;
  Preferences nvs;
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_ESP32_NVS_H