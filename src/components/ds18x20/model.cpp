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

bool DS18X20Model::EncodeDS18x20Added(char *onewire_pin, bool is_init) {
  // Fill the Ds18x20Added message
  _msg_DS18x20Added = wippersnapper_ds18x20_Ds18x20Added_init_zero;
  _msg_DS18x20Added.is_initialized = is_init;
  strcpy(_msg_DS18x20Added.onewire_pin, onewire_pin);

  // Encode the Ds18x20Added message
  size_t sz_aio_event_msg;
  if (!pb_get_encoded_size(&sz_aio_event_msg,
                           wippersnapper_ds18x20_Ds18x20Added_fields,
                           &_msg_DS18x20Added))
    return false;

  uint8_t buf[sz_aio_event_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_ds18x20_Ds18x20Added_fields,
                   &_msg_DS18x20Added);
}