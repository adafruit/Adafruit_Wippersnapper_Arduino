/*!
 * @file src/components/analog/model.cpp
 *
 * Interfaces for the analog.proto API
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
    @brief  AnalogModel constructor
*/
AnalogModel::AnalogModel() {
  memset(&_msg_AnalogAdd, 0, sizeof(_msg_AnalogAdd));
  memset(&_msg_AnalogRemove, 0, sizeof(_msg_AnalogRemove));
  memset(&_msg_AnalogEvent, 0, sizeof(_msg_AnalogEvent));
  // no-op
}

/*!
    @brief  AnalogModel destructor
*/
AnalogModel::~AnalogModel() {
  memset(&_msg_AnalogAdd, 0, sizeof(_msg_AnalogAdd));
  memset(&_msg_AnalogRemove, 0, sizeof(_msg_AnalogRemove));
  memset(&_msg_AnalogEvent, 0, sizeof(_msg_AnalogEvent));
}

/*!
    @brief  Decodes an AnalogAdd message from a stream into an
            AnalogAdd message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
bool AnalogModel::decodeAnalogAdd(pb_istream_t *stream) {
  // Zero-out the AnalogAdd message struct. to ensure we don't have any old
  // data
  memset(&_msg_AnalogAdd, 0, sizeof(_msg_AnalogAdd));
  // Decode the stream into a AnalogAdd message
  return pb_decode(stream, ws_analog_Add_fields, &_msg_AnalogAdd);
}

/*!
    @brief  Gets an AnalogAdd message struct.
    @return Pointer to an AnalogAdd message struct.
*/
ws_analog_Add *AnalogModel::getAnalogAddMsg() { return &_msg_AnalogAdd; }

/*!
    @brief  Decodes an AnalogRemove message from a stream into an
            AnalogRemove message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
bool AnalogModel::decodeAnalogRemove(pb_istream_t *stream) {
  // Zero-out the AnalogRemove message struct. to ensure we don't have any old
  // data
  memset(&_msg_AnalogRemove, 0, sizeof(_msg_AnalogRemove));
  // Decode the stream into a AnalogRemove message
  return pb_decode(stream, ws_analog_Remove_fields, &_msg_AnalogRemove);
}

/*!
    @brief  Gets an AnalogRemove message struct.
    @return Pointer to an AnalogRemove message struct.
*/
ws_analog_Remove *AnalogModel::GetAnalogRemoveMsg() {
  return &_msg_AnalogRemove;
}

/*!
    @brief  Gets an AnalogEvent message struct.
    @return Pointer to an AnalogEvent message struct.
*/
ws_analog_Event *AnalogModel::getAnalogEvent() { return &_msg_AnalogEvent; }

/*!
    @brief  Gets an Analog DeviceToBroker message struct.
    @return Pointer to an Analog D2B message struct.
*/
ws_analog_D2B *AnalogModel::getAnalogD2b() { return &_msg_AnalogD2B; }

/*!
    @brief  Encodes an AnalogEvent message.
    @param pin_name
           The pin's name string (e.g. "A0" or "EXP_0x48_0").
    @param pin_value
           The value of the pin.
    @param read_type
           The type of sensor event to encode.
    @return True if successful, False otherwise.
*/
bool AnalogModel::encodeAnalogEvent(const char *pin_name, float pin_value,
                                    ws_sensor_Type read_type) {
  // Initialize the AnalogEvent message to default values
  memset(&_msg_AnalogEvent, 0, sizeof(_msg_AnalogEvent));
  // Fill the AnalogEvent message's fields
  strncpy(_msg_AnalogEvent.pin, pin_name, sizeof(_msg_AnalogEvent.pin));
  _msg_AnalogEvent.has_value = true;
  _msg_AnalogEvent.value.type = read_type;
  _msg_AnalogEvent.value.which_value = ws_sensor_Event_float_value_tag;
  _msg_AnalogEvent.value.value.float_value = pin_value;

  // Wrap the event in the D2B envelope
  memset(&_msg_AnalogD2B, 0, sizeof(_msg_AnalogD2B));
  _msg_AnalogD2B.which_payload = ws_analog_D2B_event_tag;
  _msg_AnalogD2B.payload.event = _msg_AnalogEvent;

  return true;
}

/*!
    @brief  Encodes an AnalogEvent message with a raw pin value.
    @param pin_name
           The pin's name string.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
bool AnalogModel::encodeAnalogEventRaw(const char *pin_name, float pin_value) {
  WS_DEBUG_PRINT("[analog] Pin: ");
  WS_DEBUG_PRINTVAR(pin_name);
  WS_DEBUG_PRINT(" | Raw Value: ");
  WS_DEBUG_PRINTLNVAR(pin_value);
  return encodeAnalogEvent(pin_name, pin_value, ws_sensor_Type_T_RAW);
}

/*!
    @brief  Encodes an AnalogEvent message with a voltage pin value.
    @param pin_name
           The pin's name string.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
bool AnalogModel::encodeAnalogEventVoltage(const char *pin_name,
                                           float pin_value) {
  WS_DEBUG_PRINT("[analog] Pin: ");
  WS_DEBUG_PRINTVAR(pin_name);
  WS_DEBUG_PRINT(" | Voltage: ");
  WS_DEBUG_PRINTLNVAR(pin_value);
  return encodeAnalogEvent(pin_name, pin_value, ws_sensor_Type_T_VOLTAGE);
}
