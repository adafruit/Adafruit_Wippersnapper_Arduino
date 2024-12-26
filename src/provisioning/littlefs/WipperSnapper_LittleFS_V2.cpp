/*!
 * @file Wippersnapper_LittleFS.cpp
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
#if defined(ARDUINO_FEATHER_ESP32) ||                                          \
    defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH) ||                                \
    defined(ARDUINO_ADAFRUIT_ITSYBITSY_ESP32) ||                               \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2) ||                              \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3) || defined(ARDUINO_ESP32_DEV) ||    \
    defined(ESP32_DEV)
#include "WipperSnapper_LittleFS_V2.h"

/**************************************************************************/
/*!
    @brief    Attempts to set up and initialize a pre-existing LittleFS
              filesystem.
*/
/**************************************************************************/
WipperSnapper_LittleFS_V2::WipperSnapper_LittleFS_V2() {
  // Attempt to initialize filesystem
  if (!LittleFS.begin()) {
    fsHalt("ERROR: Failure initializing LittleFS!",
           WS_LED_STATUS_WAITING_FOR_REG_MSG);
  }
}

/**************************************************************************/
/*!
    @brief    Destructor for LittleFS
*/
/**************************************************************************/
WipperSnapper_LittleFS_V2::~WipperSnapper_LittleFS_V2() { LittleFS.end(); }

/**************************************************************************/
/*!
    @brief    Locates, opens and parses the WipperSnapper secrets file
              on the LittleFS filesystem.
*/
/**************************************************************************/
void WipperSnapper_LittleFS_V2::parseSecrets() {
  // Check if `secrets.json` file exists on FS
  if (!LittleFS.exists("/secrets.json")) {
    fsHalt("ERROR: No secrets.json found on filesystem - did you upload "
           "credentials?");
  }

  // Attempt to open secrets.json file for reading
  File secretsFile = LittleFS.open("/secrets.json", "r");
  if (!secretsFile) {
    fsHalt("ERROR: Could not open secrets.json file for reading!");
  }

  // Attempt to deserialize the file's JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, secretsFile);
  if (error) {
    fsHalt(String("ERROR: deserializeJson() failed with code ") +
           error.c_str());
  }
  if (doc.containsKey("network_type_wifi")) {
    // set default network config
    convertFromJson(doc["network_type_wifi"], WS._config.network);

    if (!doc["network_type_wifi"].containsKey("alternative_networks")) {
      // do nothing extra, we already have the only network
      WS_DEBUG_PRINTLN("Found single wifi network in secrets.json");

    } else if (doc["network_type_wifi"]["alternative_networks"]
                   .is<JsonArray>()) {

      WS_DEBUG_PRINTLN("Found multiple wifi networks in secrets.json");
      // Parse network credentials from array in secrets
      JsonArray altnetworks = doc["network_type_wifi"]["alternative_networks"];
      int8_t altNetworkCount = (int8_t)altnetworks.size();
      WS_DEBUG_PRINT("Network count: ");
      WS_DEBUG_PRINTLN(altNetworkCount);
      if (altNetworkCount == 0) {
        fsHalt("ERROR: No alternative network entries found under "
               "network_type_wifi.alternative_networks in secrets.json!");
      }
      // check if over 3, warn user and take first three
      for (int i = 0; i < altNetworkCount; i++) {
        if (i >= 3) {
          WS_DEBUG_PRINT("WARNING: More than 3 networks in secrets.json, "
                         "only the first 3 will be used. Not using ");
          WS_DEBUG_PRINTLN(altnetworks[i]["network_ssid"].as<const char *>());
          break;
        }
        convertFromJson(altnetworks[i], WS._multiNetworks[i]);
        WS_DEBUG_PRINT("Added SSID: ");
        WS_DEBUG_PRINTLN(WS._multiNetworks[i].ssid);
        WS_DEBUG_PRINT("PASS: ");
        WS_DEBUG_PRINTLN(WS._multiNetworks[i].pass);
      }
      WS._isWiFiMulti = true;
    } else {
      fsHalt("ERROR: Unrecognised value type for "
             "network_type_wifi.alternative_networks in secrets.json!");
    }
  } else {
    fsHalt("ERROR: Could not find network_type_wifi in secrets.json!");
  }

  // Extract a config struct from the JSON document
  WS._config = doc.as<secretsConfig>();

  // Validate the config struct is not filled with default values
  if (strcmp(WS._config.aio_user, "YOUR_IO_USERNAME_HERE") == 0 ||
      strcmp(WS._config.aio_key, "YOUR_IO_KEY_HERE") == 0) {
    fsHalt(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!\n");
  }

  if (strcmp(WS._config.network.ssid, "YOUR_WIFI_SSID_HERE") == 0 ||
      strcmp(WS._config.network.pass, "YOUR_WIFI_PASS_HERE") == 0) {
    fsHalt("ERROR: Invalid network credentials in secrets.json! TO FIX: Please "
           "change network_ssid and network_password to match your Adafruit IO "
           "credentials!\n");
  }

  // Close the file
  secretsFile.close();

  // Stop LittleFS, we no longer need it
  LittleFS.end();
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
    @param    msg
                Error message to print to serial console.
*/
/**************************************************************************/
void WipperSnapper_LittleFS_V2::fsHalt(String msg,
                                       ws_led_status_t status_state) {
  statusLEDSolid(status_state);
  while (1) {
    WS_DEBUG_PRINTLN("Fatal Error: Halted execution!");
    WS_DEBUG_PRINTLN(msg.c_str());
    delay(1000);
    yield();
  }
}

/**************************************************************************/
/*!
    @brief    Attempts to obtain the hardware's CS pin from the
              config.json file.
*/
/**************************************************************************/
void WipperSnapper_LittleFS_V2::GetSDCSPin() {
  // Attempt to open and deserialize the config.json file
  File file_cfg = LittleFS.open("/config.json");
  if (!file_cfg)
    WsV2.pin_sd_cs = 255;

  DeserializationError error = deserializeJson(WsV2._config_doc, file_cfg);
  if (error) {
    file_cfg.close();
    WsV2.pin_sd_cs = 255;
  }

  // Parse config.json and save the SD CS pin
  JsonObject exportedFromDevice = WsV2._config_doc["exportedFromDevice"];
  WsV2.pin_sd_cs = exportedFromDevice["sd_cs_pin"] | 255;
  file_cfg.close();
}

#endif