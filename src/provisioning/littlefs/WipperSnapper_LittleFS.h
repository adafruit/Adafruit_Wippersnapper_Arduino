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
class Wippersnapper_LittleFS {
public:
  Wippersnapper_LittleFS();
  ~Wippersnapper_LittleFS();

  void parseSecrets();
  void fsHalt();
  bool setNetwork; /*!< True if a network interface type was set up, False otherwise. */
private:

};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_LITTLEFS_H