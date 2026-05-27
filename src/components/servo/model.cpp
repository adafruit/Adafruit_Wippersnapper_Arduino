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
  memset(&_msg_servo_remove, 0, sizeof(_msg_servo_remove));
  memset(&_msg_servo_write, 0, sizeof(_msg_servo_write));
}

/*!
    @brief  Destructor
*/
ServoModel::~ServoModel() {
  memset(&_msg_servo_add, 0, sizeof(_msg_servo_add));
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
  return pb_decode(stream, ws_servo_Add_fields, &_msg_servo_add);
}

/*!
    @brief  Returns a pointer to the ServoAdd message
    @returns Pointer to ServoAdd message
*/
ws_servo_Add *ServoModel::GetServoAddMsg() { return &_msg_servo_add; }

/*!
    @brief  Decodes a ServoRemove message from a pb_istream_t
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
bool ServoModel::DecodeServoRemove(pb_istream_t *stream) {
  memset(&_msg_servo_remove, 0, sizeof(_msg_servo_remove));
  return pb_decode(stream, ws_servo_Remove_fields, &_msg_servo_remove);
}

/*!
    @brief  Returns a pointer to the ServoRemove message
    @returns Pointer to ServoRemove message
*/
ws_servo_Remove *ServoModel::GetServoRemoveMsg() { return &_msg_servo_remove; }

/*!
    @brief  Decodes a ServoWrite message from a pb_istream_t
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
bool ServoModel::DecodeServoWrite(pb_istream_t *stream) {
  memset(&_msg_servo_write, 0, sizeof(_msg_servo_write));
  return pb_decode(stream, ws_servo_Write_fields, &_msg_servo_write);
}

/*!
    @brief  Returns a pointer to the ServoWrite message
    @returns Pointer to ServoWrite message
*/
ws_servo_Write *ServoModel::GetServoWriteMsg() { return &_msg_servo_write; }