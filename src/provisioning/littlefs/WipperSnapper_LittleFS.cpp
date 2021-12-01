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
  setNetwork = false;
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

  // Get io username
  io_username = _doc["io_username"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_username == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_username value in secrets.json!");
    while (1) {
      WS.statusLEDBlink(WS_LED_STATUS_FS_WRITE);
      yield();
    }
  }

  // check if username is from templated json
  if (_doc["io_username"] == "YOUR_IO_USERNAME_HERE") {
    fsHalt();
  }

  // Get io key
  io_key = _doc["io_key"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_key == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_key value in secrets.json!");
    fsHalt();
  }

  // next, we detect the network interface from the `secrets.json`
  WS_DEBUG_PRINTLN("Attempting to find network interface...");

  const char *network_type_wifi_native_network_ssid =
      _doc["network_type_wifi_native"]["network_ssid"];
  if (network_type_wifi_native_network_ssid == nullptr) {
    WS_DEBUG_PRINTLN(
        "Network interface is not native WiFi, checking next type...");
  } else {
    WS_DEBUG_PRINTLN("Network Interface Found: Native WiFi (ESP32S2, ESP32)");
    WS_DEBUG_PRINTLN("Setting network:");
    // Parse network password
    const char *network_type_wifi_native_network_password =
        _doc["network_type_wifi_native"]["network_password"];
    // validation
    if (network_type_wifi_native_network_password == nullptr) {
      WS_DEBUG_PRINTLN(
          "ERROR: invalid network_type_wifi_native_network_password value in "
          "secrets.json!");
      fsHalt();
    }
    // check if SSID is from template (not entered)
    if (_doc["network_type_wifi_native"]["network_password"] ==
        "YOUR_WIFI_SSID_HERE") {
      fsHalt();
    }

    // Set WiFi configuration with parsed values
    WS._network_ssid = network_type_wifi_native_network_ssid;
    WS._network_pass = network_type_wifi_native_network_password;
    setNetwork = true;
  }

  // Was a network_type detected in the configuration file?
  if (!setNetwork) {
    WS_DEBUG_PRINTLN("ERROR: Network interface not detected in secrets.json file.");
    fsHalt();
  }

  // clear the document and release all memory from the memory pool
  _doc.clear();

  // close the tempFile
  secretsFile.close();
}

void WipperSnapper_LittleFS::fsHalt() {
  while (1) {
    WS.statusLEDBlink(WS_LED_STATUS_FS_WRITE);
    delay(1000);
    yield();
  }
}