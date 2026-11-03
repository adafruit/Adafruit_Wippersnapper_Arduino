/*!
 * @file src/components/digital/model.cpp
 *
 * Model for the digital.proto message.
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
    @brief  DigitalModel constructor
*/
DigitalModel::DigitalModel() {
  memset(&_msg_digital_add, 0, sizeof(_msg_digital_add));
  memset(&_msg_digital_remove, 0, sizeof(_msg_digital_remove));
  memset(&_msg_digital_event, 0, sizeof(_msg_digital_event));
  memset(&_msg_digital_write, 0, sizeof(_msg_digital_write));
  // no-op
}

/*!
    @brief  DigitalModel destructor
*/
DigitalModel::~DigitalModel() {
  memset(&_msg_digital_add, 0, sizeof(_msg_digital_add));
  memset(&_msg_digital_remove, 0, sizeof(_msg_digital_remove));
  memset(&_msg_digital_event, 0, sizeof(_msg_digital_event));
  memset(&_msg_digital_write, 0, sizeof(_msg_digital_write));
}

/*!
    @brief  Parses a DigitalAdd message.
    @return DigitalAdd message object.
*/
ws_digital_Add *DigitalModel::GetDigitalAddMsg() { return &_msg_digital_add; }

/*!
    @brief  Parses a DigitalRemove message.
    @param  stream
            The nanopb input stream.
    @return DigitalRemove message object.
*/
bool DigitalModel::DecodeDigitalRemove(pb_istream_t *stream) {
  // Zero-out the DigitalRemove message struct. to ensure we don't have any
  // old data
  memset(&_msg_digital_remove, 0, sizeof(_msg_digital_remove));

  // Decode the stream into a DigitalRemove message
  return pb_decode(stream, ws_digital_Remove_fields, &_msg_digital_remove);
}

/*!
    @brief  Gets a DigitalWrite message.
    @return DigitalWrite message object.
*/
ws_digital_Write *DigitalModel::GetDigitalWriteMsg() {
  return &_msg_digital_write;
}

/*!
    @brief  Gets a DigitalEvent message.
    @return DigitalEvent message object.
*/
ws_digital_Event *DigitalModel::GetDigitalEventMsg() {
  return &_msg_digital_event;
}

/*!
    @brief  Gets a Digital DeviceToBroker message struct.
    @return Pointer to a Digital D2B message struct.
*/
ws_digital_D2B *DigitalModel::GetDigitalD2B() { return &_msg_digital_d2b; }

/*!
    @brief  Decodes a DigitalAdd message into the _msg_digital_add object
            from a nanopb stream.
    @param  stream
            The nanopb input stream.
    @return True if the DigitalAdd message was successfully decoded.
*/
bool DigitalModel::DecodeDigitalAdd(pb_istream_t *stream) {
  // Zero-out the DigitalAdd message struct. to ensure we don't have any old
  // data
  memset(&_msg_digital_add, 0, sizeof(_msg_digital_add));

  // Decode the stream into a DigitalAdd message
  return pb_decode(stream, ws_digital_Add_fields, &_msg_digital_add);
}

/*!
    @brief  Decodes a DigitalWrite message into the _msg_digital_write
            object from a nanopb stream.
    @param  stream
            The nanopb input stream.
    @return True if the DigitalWrite message was successfully decoded.
*/
bool DigitalModel::DecodeDigitalWrite(pb_istream_t *stream) {
  // Zero-out the DigitalWrite message struct. to ensure we don't have any old
  // data
  memset(&_msg_digital_write, 0, sizeof(_msg_digital_write));
  // Decode the stream into a DigitalWrite message
  return pb_decode(stream, ws_digital_Write_fields, &_msg_digital_write);
}

/*!
    @brief  Encodes a DigitalEvent message into the
            _msg_digital_event object.
    @param  pin_name
            The pin's name.
    @param  value
            The pin's value.
    @return True if the DigitalEvent message was successfully encoded.
            False if encoding resulted in a failure.
*/
bool DigitalModel::EncodeDigitalEvent(char *pin_name, bool value) {
  // Initialize the DigitalEvent
  memset(&_msg_digital_event, 0, sizeof(_msg_digital_event));
  // Fill the DigitalEvent
  strncpy(_msg_digital_event.pin, pin_name, sizeof(_msg_digital_event.pin));
  _msg_digital_event.has_value = true;
  // Fill the DigitalEvent's SensorEvent sub-message
  _msg_digital_event.value.type = ws_sensor_Type_T_BOOLEAN;
  _msg_digital_event.value.which_value = ws_sensor_Event_bool_value_tag;
  _msg_digital_event.value.value.bool_value = value;

  // Wrap the event in the D2B envelope
  memset(&_msg_digital_d2b, 0, sizeof(_msg_digital_d2b));
  _msg_digital_d2b.which_payload = ws_digital_D2B_event_tag;
  _msg_digital_d2b.payload.event = _msg_digital_event;

  return true;
}