/*!
 * @file src/components/servo/model.cpp
 *
 * Model for the servo.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "model.h"

/*!
    @brief  Constructor
*/
ServoModel::ServoModel() {
  memset(&_msg_servo_add, 0, sizeof(_msg_servo_add));
  memset(&_msg_servo_added, 0, sizeof(_msg_servo_added));
  memset(&_msg_servo_remove, 0, sizeof(_msg_servo_remove));
  memset(&_msg_servo_write, 0, sizeof(_msg_servo_write));
}

/*!
    @brief  Destructor
*/
ServoModel::~ServoModel() {
  memset(&_msg_servo_add, 0, sizeof(_msg_servo_add));
  memset(&_msg_servo_added, 0, sizeof(_msg_servo_added));
  memset(&_msg_servo_remove, 0, sizeof(_msg_servo_remove));
  memset(&_msg_servo_write, 0, sizeof(_msg_servo_write));
}

/*!
    @brief  Decodes a ServoAdd message from a pb_istream_t
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
bool ServoModel::DecodeServoAdd(pb_istream_t *stream) {
  memset(&_msg_servo_add, 0, sizeof(_msg_servo_add));
  return pb_decode(stream, wippersnapper_servo_ServoAdd_fields,
                   &_msg_servo_add);
}

/*!
    @brief  Returns a pointer to the ServoAdd message
    @returns Pointer to ServoAdd message
*/
wippersnapper_servo_ServoAdd *ServoModel::GetServoAddMsg() {
  return &_msg_servo_add;
}

/*!
    @brief Encodes a ServoAdded message
    @param pin_name
           Name of the pin
    @param did_attach
           True if a servo was attached to the pin successfully,
           False otherwise
    @returns True if successful, False otherwise
*/
bool ServoModel::EncodeServoAdded(char *pin_name, bool did_attach) {
  // Fill the message
  memset(&_msg_servo_added, 0, sizeof(_msg_servo_added));
  _msg_servo_added.attach_success = did_attach;
  strncpy(_msg_servo_added.servo_pin, pin_name,
          sizeof(_msg_servo_added.servo_pin) - 1);
  // Encode it!
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, wippersnapper_servo_ServoAdded_fields,
                           &_msg_servo_added))
    return false;
  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_servo_ServoAdded_fields,
                   &_msg_servo_added);
}

/*!
    @brief  Returns a pointer to the ServoAdded message
    @returns Pointer to ServoAdded message
*/
wippersnapper_servo_ServoAdded *ServoModel::GetServoAddedMsg() {
  return &_msg_servo_added;
}

/*!
    @brief  Decodes a ServoRemove message from a pb_istream_t
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
bool ServoModel::DecodeServoRemove(pb_istream_t *stream) {
  memset(&_msg_servo_remove, 0, sizeof(_msg_servo_remove));
  return pb_decode(stream, wippersnapper_servo_ServoRemove_fields,
                   &_msg_servo_remove);
}

/*!
    @brief  Returns a pointer to the ServoRemove message
    @returns Pointer to ServoRemove message
*/
wippersnapper_servo_ServoRemove *ServoModel::GetServoRemoveMsg() {
  return &_msg_servo_remove;
}

/*!
    @brief  Decodes a ServoWrite message from a pb_istream_t
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
bool ServoModel::DecodeServoWrite(pb_istream_t *stream) {
  memset(&_msg_servo_write, 0, sizeof(_msg_servo_write));
  return pb_decode(stream, wippersnapper_servo_ServoWrite_fields,
                   &_msg_servo_write);
}

/*!
    @brief  Returns a pointer to the ServoWrite message
    @returns Pointer to ServoWrite message
*/
wippersnapper_servo_ServoWrite *ServoModel::GetServoWriteMsg() {
  return &_msg_servo_write;
}