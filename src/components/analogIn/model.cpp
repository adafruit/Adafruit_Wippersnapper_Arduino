/*!
 * @file src/components/analogIn/model.cpp
 *
 * Interfaces for the analogin.proto API
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
    @brief  AnalogInModel constructor
*/
AnalogInModel::AnalogInModel() {
  memset(&_msg_AnalogInAdd, 0, sizeof(_msg_AnalogInAdd));
  memset(&_msg_AnalogInRemove, 0, sizeof(_msg_AnalogInRemove));
  memset(&_msg_AnalogInEvent, 0, sizeof(_msg_AnalogInEvent));
  // no-op
}

/*!
    @brief  AnalogInModel destructor
*/
AnalogInModel::~AnalogInModel() {
  memset(&_msg_AnalogInAdd, 0, sizeof(_msg_AnalogInAdd));
  memset(&_msg_AnalogInRemove, 0, sizeof(_msg_AnalogInRemove));
  memset(&_msg_AnalogInEvent, 0, sizeof(_msg_AnalogInEvent));
}

/*!
    @brief  Decodes an AnalogInAdd message from a stream into an
            AnalogInAdd message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
bool AnalogInModel::DecodeAnalogInAdd(pb_istream_t *stream) {
  // Zero-out the AnalogInAdd message struct. to ensure we don't have any old
  // data
  memset(&_msg_AnalogInAdd, 0, sizeof(_msg_AnalogInAdd));
  // Decode the stream into a AnalogInAdd message
  return pb_decode(stream, ws_analogin_Add_fields, &_msg_AnalogInAdd);
}

/*!
    @brief  Gets an AnalogInAdd message struct.
    @return Pointer to an AnalogInAdd message struct.
*/
ws_analogin_Add *AnalogInModel::GetAnalogInAddMsg() {
  return &_msg_AnalogInAdd;
}

/*!
    @brief  Decodes an AnalogInRemove message from a stream into an
            AnalogInRemove message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
bool AnalogInModel::DecodeAnalogInRemove(pb_istream_t *stream) {
  // Zero-out the AnalogInRemove message struct. to ensure we don't have any old
  // data
  memset(&_msg_AnalogInRemove, 0, sizeof(_msg_AnalogInRemove));
  // Decode the stream into a AnalogInRemove message
  return pb_decode(stream, ws_analogin_Remove_fields, &_msg_AnalogInRemove);
}

/*!
    @brief  Gets an AnalogInRemove message struct.
    @return Pointer to an AnalogInRemove message struct.
*/
ws_analogin_Remove *AnalogInModel::GetAnalogInRemoveMsg() {
  return &_msg_AnalogInRemove;
}

/*!
    @brief  Gets an AnalogInEvent message struct.
    @return Pointer to an AnalogInEvent message struct.
*/
ws_analogin_Event *AnalogInModel::GetAnalogInEvent() {
  return &_msg_AnalogInEvent;
}

/*!
    @brief  Gets an AnalogIn DeviceToBroker message struct.
    @return Pointer to an AnalogIn D2B message struct.
*/
ws_analogin_D2B *AnalogInModel::GetAnalogInD2B() { return &_msg_AnalogInD2B; }

/*!
    @brief  Encodes an AnalogInEvent message.
    @param pin
           The requested pin's number.
    @param pin_value
           The value of the pin.
    @param read_type
           The type of sensor event to encode.
    @return True if successful, False otherwise.
*/
bool AnalogInModel::EncodeAnalogInEvent(uint8_t pin, float pin_value,
                                        ws_sensor_Type read_type) {
  // Convert pin number to pin name string
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  // Initialize the AnalogInEvent message to default values
  memset(&_msg_AnalogInEvent, 0, sizeof(_msg_AnalogInEvent));
  // Fill the AnalogInEvent message's fields
  strncpy(_msg_AnalogInEvent.pin_name, c_pin_name,
          sizeof(_msg_AnalogInEvent.pin_name));
  _msg_AnalogInEvent.has_value = true;
  _msg_AnalogInEvent.value.type = read_type;
  _msg_AnalogInEvent.value.which_value = ws_sensor_Event_float_value_tag;
  _msg_AnalogInEvent.value.value.float_value = pin_value;

  // Wrap the event in the D2B envelope
  memset(&_msg_AnalogInD2B, 0, sizeof(_msg_AnalogInD2B));
  _msg_AnalogInD2B.which_payload = ws_analogin_D2B_event_tag;
  _msg_AnalogInD2B.payload.event = _msg_AnalogInEvent;

  return true;
}

/*!
    @brief  Encodes an AnalogInEvent message with a raw pin value.
    @param pin
           The requested pin's number.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
bool AnalogInModel::EncodeAnalogInEventRaw(uint8_t pin, float pin_value) {
  WS_DEBUG_PRINT("[analogin] Pin: A");
  WS_DEBUG_PRINTVAR(pin);
  WS_DEBUG_PRINT(" | Raw Value: ");
  WS_DEBUG_PRINTLNVAR(pin_value);
  return EncodeAnalogInEvent(pin, pin_value, ws_sensor_Type_T_RAW);
}

/*!
    @brief  Encodes an AnalogInEvent message with a voltage pin value.
    @param pin
           The requested pin's number.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
bool AnalogInModel::EncodeAnalogInEventVoltage(uint8_t pin, float pin_value) {
  WS_DEBUG_PRINT("[analogin] Pin: A");
  WS_DEBUG_PRINTVAR(pin);
  WS_DEBUG_PRINT(" | Voltage: ");
  WS_DEBUG_PRINTLNVAR(pin_value);
  return EncodeAnalogInEvent(pin, pin_value, ws_sensor_Type_T_VOLTAGE);
}
