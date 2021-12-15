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
#ifndef WIPPERSNAPPER_ESP32_nvs_H
#define WIPPERSNAPPER_ESP32_nvs_H

#include "Wippersnapper.h"
#include <Preferences.h>

class Wippersnapper;
/**************************************************************************/
/*!
    @brief  Class that provides access to the ESP32's non-volatile-storage
              provisioning workflow.
*/
/**************************************************************************/
class Wippersnapper_ESP32_nvs {
public:
  Wippersnapper_ESP32_nvs();
  ~Wippersnapper_ESP32_nvs();

  bool validateNVSConfig();
  bool setNVSConfig();

  Preferences nvs; ///< Provides access to ESP32's Non-Volatile Storage

private:
  String _ssid;
  String _ssidPass;
  String _aioUser;
  String _aioPass;
  String _aioURL;
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_ESP32_NVS_H