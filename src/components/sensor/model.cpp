/*!
 * @file model.cpp
 *
 * Model for the sensor.proto message.
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

SensorModel::SensorModel() {
  _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_default;
}

SensorModel::~SensorModel() {}

bool SensorModel::decodeSensorEvent(pb_istream_t *stream) {
  // Decode the stream into theSensorEvent message
  if (!pb_decode(stream, wippersnapper_sensor_SensorEvent_fields,
                 &_msg_sensor_event)) {
    return false;
  }
  return true;
}

void SensorModel::clearSensorEvent() {
  _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_default;
}