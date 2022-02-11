/*!
 * @file Wippersnapper_LittleFS.h
 *
 * Interfaces with LittleFS filesystem for ESP32, ESP8266 platforms.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_LITTLEFS_H
#define WIPPERSNAPPER_LITTLEFS_H

#include "Wippersnapper.h"
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>

// forward decl.
class Wippersnapper;

/***************************************************************************/
/*!
    @brief  Class that handles WipperSnapper's LittleFS filesystem.
*/
/***************************************************************************/
class WipperSnapper_LittleFS {
public:
  WipperSnapper_LittleFS();
  ~WipperSnapper_LittleFS();

  void parseSecrets();
  void fsHalt();

private:
  // NOTE: calculated capacity with maximum
  // length of usernames/passwords/tokens
  // is 382 bytes, rounded to nearest power of 2.
  StaticJsonDocument<512> _doc; /*!< Json configuration file */
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_LITTLEFS_H