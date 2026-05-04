/*!
 * @file src/components/analogIO/model.cpp
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

/*!
    @brief  AnalogIOModel constructor
*/
AnalogIOModel::AnalogIOModel() {
  memset(&_msg_AnalogioAdd, 0, sizeof(_msg_AnalogioAdd));
  memset(&_msg_AnalogioRemove, 0, sizeof(_msg_AnalogioRemove));
  memset(&_msg_AnalogioEvent, 0, sizeof(_msg_AnalogioEvent));
  // no-op
}

/*!
    @brief  AnalogIOModel destructor
*/
AnalogIOModel::~AnalogIOModel() {
  memset(&_msg_AnalogioAdd, 0, sizeof(_msg_AnalogioAdd));
  memset(&_msg_AnalogioRemove, 0, sizeof(_msg_AnalogioRemove));
  memset(&_msg_AnalogioEvent, 0, sizeof(_msg_AnalogioEvent));
}

/*!
    @brief  Decodes an AnalogIOAdd message from a stream into an
            AnalogIOAdd message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
bool AnalogIOModel::DecodeAnalogIOAdd(pb_istream_t *stream) {
  // Zero-out the AnalogIOAdd message struct. to ensure we don't have any old
  // data
  memset(&_msg_AnalogioAdd, 0, sizeof(_msg_AnalogioAdd));
  // Decode the stream into a AnalogIOAdd message
  return pb_decode(stream, ws_analogio_Add_fields, &_msg_AnalogioAdd);
}

/*!
    @brief  Gets an AnalogIOAdd message struct.
    @return Pointer to an AnalogIOAdd message struct.
*/
ws_analogio_Add *AnalogIOModel::GetAnalogIOAddMsg() {
  return &_msg_AnalogioAdd;
}

/*!
    @brief  Decodes an AnalogIORemove message from a stream into an
            AnalogIORemove message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
bool AnalogIOModel::DecodeAnalogIORemove(pb_istream_t *stream) {
  // Zero-out the AnalogIORemove message struct. to ensure we don't have any old
  // data
  memset(&_msg_AnalogioRemove, 0, sizeof(_msg_AnalogioRemove));
  // Decode the stream into a AnalogIORemove message
  return pb_decode(stream, ws_analogio_Remove_fields, &_msg_AnalogioRemove);
}

/*!
    @brief  Gets an AnalogIORemove message struct.
    @return Pointer to an AnalogIORemove message struct.
*/
ws_analogio_Remove *AnalogIOModel::GetAnalogIORemoveMsg() {
  return &_msg_AnalogioRemove;
}

/*!
    @brief  Gets an AnalogIOEvent message struct.
    @return Pointer to an AnalogIOEvent message struct.
*/
ws_analogio_Event *AnalogIOModel::GetAnalogIOEvent() {
  return &_msg_AnalogioEvent;
}

/*!
    @brief  Gets an AnalogIO DeviceToBroker message struct.
    @return Pointer to an AnalogIO D2B message struct.
*/
ws_analogio_D2B *AnalogIOModel::GetAnalogIOD2B() { return &_msg_AnalogioD2B; }

/*!
    @brief  Encodes an AnalogIOEvent message.
    @param pin
           The requested pin's number.
    @param pin_value
           The value of the pin.
    @param read_type
           The type of sensor event to encode.
    @return True if successful, False otherwise.
*/
bool AnalogIOModel::EncodeAnalogIOEvent(uint8_t pin, float pin_value,
                                        ws_sensor_Type read_type) {
  // Convert pin number to pin name string
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  // Initialize the AnalogIOEvent message to default values
  memset(&_msg_AnalogioEvent, 0, sizeof(_msg_AnalogioEvent));
  // Fill the AnalogIOEvent message's fields
  strncpy(_msg_AnalogioEvent.pin_name, c_pin_name,
          sizeof(_msg_AnalogioEvent.pin_name));
  _msg_AnalogioEvent.has_value = true;
  _msg_AnalogioEvent.value.type = read_type;
  _msg_AnalogioEvent.value.which_value = ws_sensor_Event_float_value_tag;
  _msg_AnalogioEvent.value.value.float_value = pin_value;

  // Wrap the event in the D2B envelope
  memset(&_msg_AnalogioD2B, 0, sizeof(_msg_AnalogioD2B));
  _msg_AnalogioD2B.which_payload = ws_analogio_D2B_event_tag;
  _msg_AnalogioD2B.payload.event = _msg_AnalogioEvent;

  return true;
}

/*!
    @brief  Encodes an AnalogIOEvent message with a raw pin value.
    @param pin
           The requested pin's number.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
bool AnalogIOModel::EncodeAnalogIOEventRaw(uint8_t pin, float pin_value) {
  WS_DEBUG_PRINT("[analogio] Pin: A");
  WS_DEBUG_PRINTVAR(pin);
  WS_DEBUG_PRINT(" | Raw Value: ");
  WS_DEBUG_PRINTLNVAR(pin_value);
  return EncodeAnalogIOEvent(pin, pin_value, ws_sensor_Type_T_RAW);
}

/*!
    @brief  Encodes an AnalogIOEvent message with a voltage pin value.
    @param pin
           The requested pin's number.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
bool AnalogIOModel::EncodeAnalogIOEventVoltage(uint8_t pin, float pin_value) {
  WS_DEBUG_PRINT("[analogio] Pin: A");
  WS_DEBUG_PRINTVAR(pin);
  WS_DEBUG_PRINT(" | Voltage: ");
  WS_DEBUG_PRINTLNVAR(pin_value);
  return EncodeAnalogIOEvent(pin, pin_value, ws_sensor_Type_T_VOLTAGE);
}