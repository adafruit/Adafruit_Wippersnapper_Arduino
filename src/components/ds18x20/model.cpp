/*!
 * @file model.cpp
 *
 * Model for the ds18x20.proto message.
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
#include "model.h"

DS18X20Model::DS18X20Model() {
  // Initialize the DS18X20 messages
  _msg_DS18x20Add = wippersnapper_ds18x20_Ds18x20Add_init_zero;
}

DS18X20Model::~DS18X20Model() {}