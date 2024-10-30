/*!
 * @file Wippersnapper_LittleFS.h
 *
 * Interfaces with LittleFS filesystem for ESP32, ESP8266 platforms.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_LITTLEFS_V2_H
#define WIPPERSNAPPER_LITTLEFS_V2_H

#include "Wippersnapper_V2.h"

#include <FS.h>
#include <LittleFS.h>

// forward decl.
class Wippersnapper_V2;

/***************************************************************************/
/*!
    @brief  Class that handles WipperSnapper's LittleFS filesystem.
*/
/***************************************************************************/
class WipperSnapper_LittleFS_V2 {
public:
  WipperSnapper_LittleFS_V2();
  ~WipperSnapper_LittleFS_V2();
  void parseSecrets();
  void fsHalt(String msg);
};
extern Wippersnapper_V2 WsV2;
#endif // WIPPERSNAPPER_LITTLEFS_H