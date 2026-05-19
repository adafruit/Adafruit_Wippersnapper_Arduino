/*!
 * @file src/components/pwm/model.cpp
 *
 * Model for the pwm.proto message.
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
    @brief  Ctor for PWMModel.
*/
PWMModel::PWMModel() {
  memset(&_msg_pwm_add, 0, sizeof(_msg_pwm_add));
  memset(&_msg_pwm_remove, 0, sizeof(_msg_pwm_remove));
  memset(&_msg_pwm_write, 0, sizeof(_msg_pwm_write));
}

/*!
    @brief  Dtor for PWMModel.
*/
PWMModel::~PWMModel() {
  memset(&_msg_pwm_add, 0, sizeof(_msg_pwm_add));
  memset(&_msg_pwm_remove, 0, sizeof(_msg_pwm_remove));
  memset(&_msg_pwm_write, 0, sizeof(_msg_pwm_write));
}

/*!
    @brief  Decodes a PWMAdd message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMAdd(pb_istream_t *stream) {
  memset(&_msg_pwm_add, 0, sizeof(_msg_pwm_add));
  return pb_decode(stream, ws_pwm_Add_fields, &_msg_pwm_add);
}

/*!
    @brief  Returns a pointer to the PWMAdd message.
    @return Pointer to the PWMAdd message.
*/
ws_pwm_Add *PWMModel::GetPWMAddMsg() { return &_msg_pwm_add; }

/*!
    @brief  Decodes a PWMRemove message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMRemove(pb_istream_t *stream) {
  memset(&_msg_pwm_remove, 0, sizeof(_msg_pwm_remove));
  return pb_decode(stream, ws_pwm_Remove_fields, &_msg_pwm_remove);
}

/*!
    @brief  Returns a pointer to the PWMRemove message.
    @return Pointer to the PWMRemove message.
*/
ws_pwm_Remove *PWMModel::GetPWMRemoveMsg() { return &_msg_pwm_remove; }

/*!
    @brief  Decodes a PWMWrite message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMWrite(pb_istream_t *stream) {
  memset(&_msg_pwm_write, 0, sizeof(_msg_pwm_write));
  return pb_decode(stream, ws_pwm_Write_fields, &_msg_pwm_write);
}

/*!
    @brief  Returns a pointer to the PWMWrite message.
    @return Pointer to the PWMWrite message.
*/
ws_pwm_Write *PWMModel::GetPWMWriteMsg() { return &_msg_pwm_write; }
