/*!
 * @file Wippersnapper_LittleFS.h
 *
 * Wippersnapper LittleFS
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
#ifndef WIPPERSNAPPER_LITTLEFS_H
#define WIPPERSNAPPER_LITTLEFS_H

#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "Wippersnapper.h"

// forward decl.
class Wippersnapper;

/***************************************************************************/
/*!
    @brief  Class that handles WipperSnapper's LittleFS filesystem for
            platforms without native USB (such as ESP32, ESP8266).
*/
/***************************************************************************/
class WipperSnapper_LittleFS {
public:
  WipperSnapper_LittleFS();
  ~WipperSnapper_LittleFS();

  void parseSecrets();
  void fsHalt();

  // Adafruit IO Configuration
  const char *io_username =
      NULL;                  /*!< Adafruit IO username, from config json. */
  const char *io_key = NULL; /*!< Adafruit IO password, from config json. */
  bool setNetwork; /*!< True if a network interface type was set up, False
                      otherwise. */

private:
  // NOTE: calculated capacity with maximum
  // length of usernames/passwords/tokens
  // is 382 bytes, rounded to nearest power of 2.
  StaticJsonDocument<512> _doc; /*!< Json configuration file */
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_LITTLEFS_H