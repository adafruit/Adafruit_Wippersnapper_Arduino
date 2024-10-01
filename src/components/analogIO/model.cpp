/*!
 * @file model.cpp
 *
 * Interfaces for the analogio.proto API
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

// TODO! Implement the AnalogIOModel class

AnalogIOModel::AnalogIOModel() {
    _msg_AnalogioAdd = wippersnapper_analogio_AnalogIOAdd_init_default;
}

AnalogIOModel::~AnalogIOModel() {}

bool AnalogIOModel::DecodeAnalogIOAdd(pb_istream_t *stream) {
    // Zero-out the AnalogIOAdd message struct. to ensure we don't have any old data
    _msg_AnalogioAdd = wippersnapper_analogio_AnalogIOAdd_init_default;

    // Decode the stream into a AnalogIOAdd message
    return pb_decode(stream, wippersnapper_analogio_AnalogIOAdd_fields, &_msg_AnalogioAdd);
}

wippersnapper_analogio_AnalogIOAdd *AnalogIOModel::GetAnalogIOAddMsg() {
    return &_msg_AnalogioAdd;
}