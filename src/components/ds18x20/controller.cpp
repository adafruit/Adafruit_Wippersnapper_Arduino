/*!
 * @file controller.cpp
 *
 * Controller for the ds18x20.proto API
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
#include "controller.h"

DS18X20Controller::DS18X20Controller() { _DS18X20_model = new DS18X20Model(); }

DS18X20Controller::~DS18X20Controller() { delete _DS18X20_model; }

bool DS18X20Controller::Handle_Ds18x20Add(pb_istream_t *stream) {
  // Attempt to decode the incoming message into a Ds18x20Add message
  if (!_DS18X20_model->DecodeDS18x20Add(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode Ds18x20Add message");
    return false;
  }
  // Pass the decoded message to the hardware
  // TODO: Brent you were here and looking at this:
  // https://github.com/pstolarz/OneWireNg/blob/master/examples/arduino/DallasTemperature/DallasTemperature.ino#L125

  return true;
}
