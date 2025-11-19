/*!
 * @file src/components/ds18x20/model.cpp
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

/*!
    @brief  DS18X20Model constructor
*/
DS18X20Model::DS18X20Model() {
  memset(&_msg_DS18x20Add, 0, sizeof(_msg_DS18x20Add));
  memset(&_msg_DS18x20Added, 0, sizeof(_msg_DS18x20Added));
  memset(&_msg_DS18x20Remove, 0, sizeof(_msg_DS18x20Remove));
  memset(&_msg_DS18x20Event, 0, sizeof(_msg_DS18x20Event));
}

/*!
    @brief  DS18X20Model destructor
*/
DS18X20Model::~DS18X20Model() {
  memset(&_msg_DS18x20Add, 0, sizeof(_msg_DS18x20Add));
  memset(&_msg_DS18x20Added, 0, sizeof(_msg_DS18x20Added));
  memset(&_msg_DS18x20Remove, 0, sizeof(_msg_DS18x20Remove));
  memset(&_msg_DS18x20Event, 0, sizeof(_msg_DS18x20Event));
}

/*!
    @brief  Attempts to decode a Ds18x20Add message from the broker.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully decoded, False otherwise.
*/
bool DS18X20Model::DecodeDS18x20Add(pb_istream_t *stream) {
  memset(&_msg_DS18x20Add, 0, sizeof(_msg_DS18x20Add));
  // Attempt to decode the stream into a Ds18x20Add message
  return pb_decode(stream, wippersnapper_ds18x20_Ds18x20Add_fields,
                   &_msg_DS18x20Add);
}

/*!
    @brief  Gets a pointer to the Ds18x20Add message.
    @return Pointer to the Ds18x20Add message.
*/
wippersnapper_ds18x20_Ds18x20Add *DS18X20Model::GetDS18x20AddMsg() {
  return &_msg_DS18x20Add;
}

/*!
    @brief  Returns a pointer to the Ds18x20Added message.
    @return Pointer to the Ds18x20Added message.
*/
wippersnapper_ds18x20_Ds18x20Added *DS18X20Model::GetDS18x20AddedMsg() {
  return &_msg_DS18x20Added;
}

/*!
    @brief  Encodes a Ds18x20Added message.
    @param  onewire_pin
            The OneWire bus pin to add.
    @param  is_init
            True if the sensor was successfully initialized,
            False otherwise.
    @return True if the message was successfully encoded,
            False otherwise.
*/
bool DS18X20Model::EncodeDS18x20Added(char *onewire_pin, bool is_init) {
  // Fill the Ds18x20Added message
  memset(&_msg_DS18x20Added, 0, sizeof(_msg_DS18x20Added));
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

/*!
    @brief  Attempts to decode a Ds18x20Remove message from the broker.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully decoded, False otherwise.
*/
bool DS18X20Model::DecodeDS18x20Remove(pb_istream_t *stream) {
  memset(&_msg_DS18x20Remove, 0, sizeof(_msg_DS18x20Remove));
  return pb_decode(stream, wippersnapper_ds18x20_Ds18x20Remove_fields,
                   &_msg_DS18x20Remove);
}

/*!
    @brief  Gets a pointer to the Ds18x20Remove message.
    @return Pointer to the Ds18x20Remove message.
*/
wippersnapper_ds18x20_Ds18x20Remove *DS18X20Model::GetDS18x20RemoveMsg() {
  return &_msg_DS18x20Remove;
}

/*!
    @brief  Gets a pointer to the Ds18x20Event message.
    @return Pointer to the Ds18x20Event message.
*/
ws_ds18x20_Event *DS18X20Model::GetDS18x20EventMsg() {
  return &_msg_DS18x20Event;
}

/*!
    @brief  Encodes a Ds18x20Event message.
    @return True if the message was successfully encoded, False otherwise.
*/
bool DS18X20Model::EncodeDs18x20Event() {
  // take the filled _msg_DS18x20Event we built in the controller and encode it
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, ws_ds18x20_Event_fields,
                           &_msg_DS18x20Event))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_ds18x20_Event_fields, &_msg_DS18x20Event);
}

/*!
    @brief  Initializes the Ds18x20Event message.
    @param  ow_pin_name
            The OneWire bus pin name.
*/
void DS18X20Model::InitDS18x20EventMsg(const char *ow_pin_name) {
  memset(&_msg_DS18x20Event, 0, sizeof(_msg_DS18x20Event));
  _msg_DS18x20Event.sensor_events_count = 0;
  strcpy(_msg_DS18x20Event.onewire_pin, ow_pin_name);
}

/*!
    @brief  Adds a "sensor event" to the Ds18x20Event message.
    @param  sensor_type
            The event's SensorType.
    @param  sensor_value
            The event's value.
*/
void DS18X20Model::addSensorEvent(ws_sensor_Type sensor_type,
                                  float sensor_value) {
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count].type =
      sensor_type;
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count]
      .which_value = wippersnapper_sensor_SensorEvent_float_value_tag;

  // Convert the float to 2 decimal places
  sensor_value = roundf(sensor_value * 100) / 100;
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count]
      .value.float_value = sensor_value;
  _msg_DS18x20Event.sensor_events_count++;
}