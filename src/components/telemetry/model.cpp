/*!
 * @file src/components/telemetry/model.cpp
 *
 * Model for the telemetry.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "model.h"

/*!
    @brief  Callback function to encode a string as a nanopb bytes field.
    @param  stream
            The nanopb output stream.
    @param  field
            The nanopb field descriptor.
    @param  arg
            Pointer to the string to encode.
    @return True if encoding was successful, False otherwise.
*/
bool TelemetryModel::encode_string_cb(pb_ostream_t *stream,
                                      const pb_field_t *field,
                                      void *const *arg) {
  const char *str = (const char *)*arg;
  if (!str) {
    return pb_encode_tag_for_field(stream, field) &&
           pb_encode_string(stream, (const uint8_t *)"", 0);
  }
  if (!pb_encode_tag_for_field(stream, field))
    return false;
  return pb_encode_string(stream, (const uint8_t *)str, strlen(str));
}

/*!
    @brief  TelemetryModel constructor
*/
TelemetryModel::TelemetryModel() {
  memset(&_msg_Add, 0, sizeof(_msg_Add));
  memset(&_msg_Remove, 0, sizeof(_msg_Remove));
  memset(&_msg_D2B, 0, sizeof(_msg_D2B));
  _str_value[0] = '\0';
}

/*!
    @brief  TelemetryModel destructor
*/
TelemetryModel::~TelemetryModel() {
  memset(&_msg_Add, 0, sizeof(_msg_Add));
  memset(&_msg_Remove, 0, sizeof(_msg_Remove));
  memset(&_msg_D2B, 0, sizeof(_msg_D2B));
}

/*!
    @brief  Attempts to decode a TelemetryAdd message from the broker.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully decoded, False otherwise.
*/
bool TelemetryModel::DecodeTelemetryAdd(pb_istream_t *stream) {
  memset(&_msg_Add, 0, sizeof(_msg_Add));
  return pb_decode(stream, ws_telemetry_Add_fields, &_msg_Add);
}

/*!
    @brief  Gets a pointer to the TelemetryAdd message.
    @return Pointer to the TelemetryAdd message.
*/
ws_telemetry_Add *TelemetryModel::GetTelemetryAddMsg() { return &_msg_Add; }

/*!
    @brief  Attempts to decode a TelemetryRemove message from the broker.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully decoded, False otherwise.
*/
bool TelemetryModel::DecodeTelemetryRemove(pb_istream_t *stream) {
  memset(&_msg_Remove, 0, sizeof(_msg_Remove));
  return pb_decode(stream, ws_telemetry_Remove_fields, &_msg_Remove);
}

/*!
    @brief  Gets a pointer to the TelemetryRemove message.
    @return Pointer to the TelemetryRemove message.
*/
ws_telemetry_Remove *TelemetryModel::GetTelemetryRemoveMsg() {
  return &_msg_Remove;
}

/*!
    @brief  Initializes the telemetry D2B Event message for a metric.
    @param  name
            The telemetry component's name.
*/
void TelemetryModel::InitEventMsg(const char *name) {
  memset(&_msg_D2B, 0, sizeof(_msg_D2B));
  _str_value[0] = '\0';
  _msg_D2B.which_payload = ws_telemetry_D2B_event_tag;
  strncpy(_msg_D2B.payload.event.name, name,
          sizeof(_msg_D2B.payload.event.name) - 1);
  _msg_D2B.payload.event.name[sizeof(_msg_D2B.payload.event.name) - 1] = '\0';
  _msg_D2B.payload.event.has_value = true;
}

/*!
    @brief  Sets the Event's value as a float.
    @param  type
            The value's SensorType.
    @param  value
            The value to report.
*/
void TelemetryModel::SetValueFloat(ws_sensor_Type type, float value) {
  ws_sensor_Event *event = &_msg_D2B.payload.event.value;
  event->type = type;
  event->which_value = ws_sensor_Event_float_value_tag;
  event->value.float_value = value;
}

/*!
    @brief  Sets the Event's value as a string (encoded as a bytes field).
    @param  type
            The value's SensorType.
    @param  value
            The string to report. Copied into the model so it stays valid
            through encoding.
*/
void TelemetryModel::SetValueString(ws_sensor_Type type, const char *value) {
  strncpy(_str_value, value, sizeof(_str_value) - 1);
  _str_value[sizeof(_str_value) - 1] = '\0';
  ws_sensor_Event *event = &_msg_D2B.payload.event.value;
  event->type = type;
  event->which_value = ws_sensor_Event_bytes_value_tag;
  event->value.bytes_value.funcs.encode = &encode_string_cb;
  event->value.bytes_value.arg = (void *)_str_value;
}

/*!
    @brief  Verifies the telemetry Event message can be encoded.
    @return True if the message was successfully encoded, False otherwise.
*/
bool TelemetryModel::EncodeEvent() {
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, ws_telemetry_D2B_fields, &_msg_D2B))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_telemetry_D2B_fields, &_msg_D2B);
}

/*!
    @brief  Gets a pointer to the telemetry D2B message.
    @return Pointer to the telemetry D2B message.
*/
ws_telemetry_D2B *TelemetryModel::GetD2B() { return &_msg_D2B; }
