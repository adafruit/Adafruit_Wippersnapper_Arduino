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

void ws_sdcard::parseConfigFile() {
  File32 file_config; // TODO: MAke this global?
  file_config = _sd.open("config.json", FILE_READ);

  JsonDocument doc;
  // TODO: Re-enable this
  // TODO: Change max input length to fit an expected/max json size
  int max_input_len = 512;

  // Attempt to de-serialize the JSON document
  // DeserializationError error = deserializeJson(doc, file_config,
  // max_input_len);
  // If the JSON document failed to deserialize - halt the running device and
  // print the error because it is not possible to continue running in offline
  // mode without a valid config file
  // if (error)
  // fsHalt("deserializeJson() failed: " + String(error.c_str()));

  // Otherwise, parse away!

  // TODO: Start by detecting which API the config file wants to parse the JSON
  // into via the "componentAPI" field
  // TODO: We only need to handle componentTypeAdd messages here

  // TODO: Can we access protobufs here?
  // create a digitalio protobuf message
  // wippersnapper_analogio_AnalogIOAdd addMsg =
  // wippersnapper_analogio_AnalogIOAdd_init_zero;
}

// Returns true if input points to a valid JSON string
bool ws_sdcard::validateJson(const char* input) {
  JsonDocument doc, filter;
  return deserializeJson(doc, input, DeserializationOption::Filter(filter)) == DeserializationError::Ok;
}

// Waits for incoming config file and parses it 
// TODO: Split out parsing into parseConfigFile() and just read here
void ws_sdcard::waitForIncomingConfigFile() {
  String serialInput = "";

  // Wait for incoming serial data
  while (true) {
    // Check if there is data available to read
    if (Serial.available() > 0) {
      // Read and append to serialInput
      char c = Serial.read();
      serialInput += c;
      // Check for EoL or end of JSON string
      if (c == '\n' || c == '}') {
        break;
      }
    }
  }

  // validate the json string
  if (validateJson(serialInput.c_str())) {
    // parse the json string
    WS_DEBUG_PRINTLN("Valid JSON string received!");
  } else {
    WS_DEBUG_PRINTLN("Invalid JSON string received!");
  }
}