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
  // Extract the OneWire pin from the message
  uint8_t pin_name = atoi(_DS18X20_model->GetDS18x20AddMsg()->onewire_pin + 1);

  // Initialize the DS18X20Hardware object
  DS18X20Hardware new_dsx_driver(pin_name);
  new_dsx_driver.setResolution(
      _DS18X20_model->GetDS18x20AddMsg()->sensor_resolution);

  // Add the DS18X20Hardware object to the vector of hardware objects
  _DS18X20_pins.push_back(new_dsx_driver);

  // TODO: We should publish back an Added message to the broker

  return true;
}
