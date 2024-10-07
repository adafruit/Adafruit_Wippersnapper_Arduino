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

DS18X20Controller::DS18X20Controller() {
  _DS18X20_model = new DS18X20Model();
  _DS18X20_hardware = new DS18X20Hardware();
}

DS18X20Controller::~DS18X20Controller() {
  delete _DS18X20_model;
  delete _DS18X20_hardware;
}

bool DS18X20Controller::Handle_Ds18x20Add(pb_istream_t *stream) {
  // TODO: This requires an implementation
  return true;
}
