/*!
 * @file ws_sdcard.cpp
 *
 * Interface for Wippersnapper's SD card filesystem.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "ws_sdcard.h"

/**************************************************************************/
/*!
    @brief    Constructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::ws_sdcard() {
  _is_sd_card_inserted = false;
#ifndef SD_CS_PIN
  return;
#endif

  // Attempt to initialize the SD card
  if (_sd.begin(SD_CS_PIN)) {
    _is_sd_card_inserted = true;
  }
}

/**************************************************************************/
/*!
    @brief    Destructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::~ws_sdcard() {
  // TODO: Close any open files
  // Then, end the SD card (ends SPI transaction)
  _sd.end();
}

/**************************************************************************/
/*!
    @brief    Checks if an SD card is inserted and mounted.
    @returns  True if an SD card is inserted and mounted, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::IsSDCardInserted() { return _is_sd_card_inserted; }

bool ws_sdcard::parseConfigFile() {
  File32 file_config; // TODO: MAke this global?
#ifndef OFFLINE_MODE_DEBUG
  file_config = _sd.open("config.json", FILE_READ);
#endif

  JsonDocument doc;
  // TODO: Change max input length to fit an expected/max json size
  int max_input_len = 512;

  // Attempt to de-serialize the JSON document
  DeserializationError error;
#ifdef ONLINE_MODE_DEBUG
  // Read the config file from the serial input buffer
  error = deserializeJson(doc, _serialInput.c_str(), max_input_len);
#else
  // Read the config file from the SD card
  WS_DEBUG_PRINTLN("Reading config file...");
// TODO - implement this
// error = deserializeJson(doc, file_config, max_input_len);
#endif

  // If the JSON document failed to deserialize - halt the running device and
  // print the error because it is not possible to continue running in offline
  // mode without a valid config file
  if (error) {
    WS_DEBUG_PRINTLN("deserializeJson() failed: " + String(error.c_str()));
    // TODO: Maybe this func should contain a bool return type for failure at
    // this point
    return false;
  }

  // Parse the "components" array
  JsonObject components = doc["components"][0];
  // Parse the PB API type
  const char *component_api_type =
      components["componentAPI"]; // ie: "analogio", "digitalio", etc.

  // TODO- maybe a Switch case to handle the different component API types but
  // for now just a simple if-else is OK
  if (strcmp(component_api_type, "digitalio") == 0) {
    // TODO - dispatch to create digitalio component protobuf message
  } else if (strcmp(component_api_type, "analogio") == 0) {
    // TODO - dispatch to create analogio component protobuf message
  } else {
    // Unknown component API type
    WS_DEBUG_PRINTLN("Unknown component API type found: " +
                     String(component_api_type));
    return false;
  }

  return true;
}

// Returns true if input points to a valid JSON string
bool ws_sdcard::validateJson(const char *input) {
  JsonDocument doc, filter;
  return deserializeJson(doc, input, DeserializationOption::Filter(filter)) ==
         DeserializationError::Ok;
}

// Note: using this for VALID json:
// {"temperature": 22.5, "humidity": 60}
// Note: using this for INVALID json:
// {"temperature": 22.5, "humidity": 60,

// Waits for incoming config file and parses it
// TODO: Split out parsing into parseConfigFile() and just read here
bool ws_sdcard::waitForSerialConfig() {
  _serialInput = ""; // Clear the serial input buffer
  WS_DEBUG_PRINTLN("Waiting for incoming JSON string...");

  // Wait for incoming serial data
  while (true) {
    // Check if there is data available to read
    if (Serial.available() > 0) {
      // Read and append to _serialInput
      char c = Serial.read();
      _serialInput += c;
      // Check for EoL or end of JSON string
      if (c == '\n' || c == '}') {
        break;
      }
    }
  }

  // Attempt to validate the string as JSON
  if (!validateJson(_serialInput.c_str())) {
    WS_DEBUG_PRINTLN("Invalid JSON string received!");
    return false;
  }

  WS_DEBUG_PRINTLN("Valid JSON string received!");
  return true;
}