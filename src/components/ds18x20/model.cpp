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

bool DS18X20Model::DecodeDS18x20Add(pb_istream_t *stream) {
  _msg_DS18x20Add = wippersnapper_ds18x20_Ds18x20Add_init_zero;
  // Attempt to decode the stream into a Ds18x20Add message
  return pb_decode(stream, wippersnapper_ds18x20_Ds18x20Add_fields,
                   &_msg_DS18x20Add);
}

wippersnapper_ds18x20_Ds18x20Add *DS18X20Model::GetDS18x20AddMsg() {
  return &_msg_DS18x20Add;
}

wippersnapper_ds18x20_Ds18x20Added *DS18X20Model::GetDS18x20AddedMsg() {
  return &_msg_DS18x20Added;
}

bool DS18X20Model::EncodeDS18x20Added(char *onewire_pin, bool is_init) {
  // Fill the Ds18x20Added message
  _msg_DS18x20Added = wippersnapper_ds18x20_Ds18x20Added_init_zero;
  _msg_DS18x20Added.is_initialized = is_init;
  strcpy(_msg_DS18x20Added.onewire_pin, onewire_pin);

  // Encode the Ds18x20Added message
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, wippersnapper_ds18x20_Ds18x20Added_fields,
                           &_msg_DS18x20Added))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_ds18x20_Ds18x20Added_fields,
                   &_msg_DS18x20Added);
}

bool DS18X20Model::DecodeDS18x20Remove(pb_istream_t *stream) {
  _msg_DS18x20Remove = wippersnapper_ds18x20_Ds18x20Remove_init_zero;
  return pb_decode(stream, wippersnapper_ds18x20_Ds18x20Remove_fields,
                   &_msg_DS18x20Remove);
}

wippersnapper_ds18x20_Ds18x20Remove *DS18X20Model::GetDS18x20RemoveMsg() {
  return &_msg_DS18x20Remove;
}

wippersnapper_ds18x20_Ds18x20Event *DS18X20Model::GetDS18x20EventMsg() {
  return &_msg_DS18x20Event;
}

bool DS18X20Model::EncodeDs18x20Event(
    char *onewire_pin, pb_size_t sensor_events_count,
    const wippersnapper_sensor_SensorEvent sensor_events[2]) {
  // Fill the Ds18x20Event message
  _msg_DS18x20Event = wippersnapper_ds18x20_Ds18x20Event_init_zero;
  strcpy(_msg_DS18x20Event.onewire_pin, onewire_pin);
  _msg_DS18x20Event.sensor_events_count = sensor_events_count;
  // Fill with sensor_events
  for (int i = 0; i < _msg_DS18x20Event.sensor_events_count; i++) {
    _msg_DS18x20Event.sensor_events[i] = sensor_events[i];
  }
  // Encode and return the Ds18x20Event message
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, wippersnapper_ds18x20_Ds18x20Event_fields,
                           &_msg_DS18x20Event))
    return false;
  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_ds18x20_Ds18x20Event_fields,
                   &_msg_DS18x20Event);
}

void DS18X20Model::InitDS18x20EventMsg() {
  _msg_DS18x20Event = wippersnapper_ds18x20_Ds18x20Event_init_zero;
  _msg_DS18x20Event.sensor_events_count = 0;
}

void DS18X20Model::addSensorEvent(wippersnapper_sensor_SensorType sensor_type,
                                  float sensor_value) {
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count].type =
      sensor_type;
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count]
      .which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count]
      .value.float_value = sensor_value;
}