/*!
 * @file Wippersnapper_LittleFS.cpp
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
#if defined(ARDUINO_FEATHER_ESP32) ||                                          \
    defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH) ||                                \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2) ||                              \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
#include "WipperSnapper_LittleFS.h"

/**************************************************************************/
/*!
    @brief    Attempts to set up and initialize a pre-existing LittleFS
              filesystem.
*/
/**************************************************************************/
WipperSnapper_LittleFS::WipperSnapper_LittleFS() {
  // Attempt to initialize filesystem
  if (!LittleFS.begin()) {
    WS_DEBUG_PRINTLN("ERROR: Failure initializing LittleFS!");
    setStatusLEDColor(RED);
    while (1)
      ;
  }
}

/**************************************************************************/
/*!
    @brief    Destructor for LittleFS
*/
/**************************************************************************/
WipperSnapper_LittleFS::~WipperSnapper_LittleFS() { LittleFS.end(); }

/**************************************************************************/
/*!
    @brief    Locates, opens and parses the WipperSnapper secrets file
              on the LittleFS filesystem.
*/
/**************************************************************************/
void WipperSnapper_LittleFS::parseSecrets() {
  // Check if `secrets.json` file exists on FS
  if (!LittleFS.exists("/secrets.json")) {
    WS_DEBUG_PRINTLN("ERROR: No secrets.json found on filesystem - did you "
                     "upload credentials?");
    fsHalt();
  }

  // Attempt to open secrets.json file for reading
  File secretsFile = LittleFS.open("/secrets.json", "r");
  if (!secretsFile) {
    WS_DEBUG_PRINTLN("ERROR: Could not open secrets.json file for reading!");
    fsHalt();
  }

  // Attempt to deserialize the file's JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, secretsFile);
  if (error) {
    WS_DEBUG_PRINT("ERROR: deserializeJson() failed with code ");
    WS_DEBUG_PRINTLN(error.c_str());
    fsHalt();
  }

  // Extract a config struct from the JSON document
  WS._config = doc.as<secretsConfig>();

  // Validate the config struct is not filled with default values
  if (strcmp(WS._config.aio_user, "YOUR_IO_USERNAME_HERE") == 0 ||
      strcmp(WS._config.aio_key, "YOUR_IO_KEY_HERE") == 0) {
    WS_DEBUG_PRINTLN(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!\n");
    fsHalt();
  }

  if (strcmp(WS._config.network.ssid, "YOUR_WIFI_SSID_HERE") == 0 ||
      strcmp(WS._config.network.pass, "YOUR_WIFI_PASS_HERE") == 0) {
    WS_DEBUG_PRINTLN("ERROR: Invalid network credentials in secrets.json! TO "
                     "FIX: Please change network_ssid and network_password to "
                     "match your Adafruit IO credentials!\n");
    fsHalt();
  }

  // Close the file
  secretsFile.close();

  // Stop LittleFS, we no longer need it
  LittleFS.end();
}

void WipperSnapper_LittleFS::fsHalt() {
  while (1) {
    statusLEDSolid(WS_LED_STATUS_FS_WRITE);
    delay(1000);
    yield();
  }
}
#endif