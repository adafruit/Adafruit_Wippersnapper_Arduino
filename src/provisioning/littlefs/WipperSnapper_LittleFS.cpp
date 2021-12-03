/*!
 * @file Wippersnapper_LittleFS.cpp
 *
 * Interacts with littleFS filesystem for non-native USB platforms.
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

#include "WipperSnapper_LittleFS.h"

// ctor
WipperSnapper_LittleFS::WipperSnapper_LittleFS() {
  // Attempt to initialize filesystem
  if (!LittleFS.begin()) {
    WS_DEBUG_PRINTLN("ERROR: Failure initializing LittleFS!");
    WS.setStatusLEDColor(RED);
    while (1)
      ;
  }

  // Check if `secrets.json` file exists on FS
  if (!LittleFS.exists("/secrets.json")) {
    WS_DEBUG_PRINTLN("ERROR: No secrets.json found on filesystem!");
    WS.setStatusLEDColor(RED);
    while (1)
      ;
  }
}

// dtor
WipperSnapper_LittleFS::~WipperSnapper_LittleFS() {
  LittleFS.end();
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
*/
/**************************************************************************/
void WipperSnapper_LittleFS::parseSecrets() {
  // open file for parsing
  File secretsFile = LittleFS.open("/secrets.json", "r");
  if (!secretsFile) {
    WS_DEBUG_PRINTLN("ERROR: Could not open secrets.json file for reading!");
    fsHalt();
  }

  // check if we can deserialize the secrets.json file
  DeserializationError err = deserializeJson(_doc, secretsFile);
  if (err) {
    WS_DEBUG_PRINT("ERROR: deserializeJson() failed with code ");
    WS_DEBUG_PRINTLN(err.c_str());
    fsHalt();
  }

  // Get IO username from JSON
  const char *io_username = _doc["io_username"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_username == nullptr)
    fsHalt();
  // set application's IO username
  WS._username = io_username;

  // Get IO key from JSON
  const char *io_key = _doc["io_key"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_key == nullptr)
    fsHalt();
  // Set applicaiton's IO key
  WS._key = io_key;

  // Parse SSID
  const char *network_type_wifi_native_network_ssid =
      _doc["network_type_wifi_native"]["network_ssid"];
  if (network_type_wifi_native_network_ssid == nullptr)
    fsHalt();

  // Parse SSID password
  const char *network_type_wifi_native_network_password = _doc["network_type_wifi_native"]["network_password"];
  // validation
  if (network_type_wifi_native_network_password == nullptr)
    fsHalt();

  // Set WiFi configuration with parsed values
  WS._network_ssid = network_type_wifi_native_network_ssid;
  WS._network_pass = network_type_wifi_native_network_password;

  // close the tempFile
  secretsFile.close();

  // clear the document and release all memory from the memory pool
  _doc.clear();

}

void WipperSnapper_LittleFS::fsHalt() {
  while (1) {
    WS.statusLEDBlink(WS_LED_STATUS_FS_WRITE);
    delay(1000);
    yield();
  }
}