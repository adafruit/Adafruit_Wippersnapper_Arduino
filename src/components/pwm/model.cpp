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
  memset(&_msg_pwm_added, 0, sizeof(_msg_pwm_added));
  memset(&_msg_pwm_remove, 0, sizeof(_msg_pwm_remove));
  /* memset(&_msg_pwm_write_duty_cycle, 0, sizeof(_msg_pwm_write_duty_cycle)); */
  /* memset(&_msg_pwm_write_frequency, 0, sizeof(_msg_pwm_write_frequency)); */
}

/*!
    @brief  Dtor for PWMModel.
*/
PWMModel::~PWMModel() {
  memset(&_msg_pwm_add, 0, sizeof(_msg_pwm_add));
  memset(&_msg_pwm_added, 0, sizeof(_msg_pwm_added));
  memset(&_msg_pwm_remove, 0, sizeof(_msg_pwm_remove));
  /* memset(&_msg_pwm_write_duty_cycle, 0, sizeof(_msg_pwm_write_duty_cycle)); */
  /* memset(&_msg_pwm_write_frequency, 0, sizeof(_msg_pwm_write_frequency)); */
}

/*!
    @brief  Decodes a PWMAdd message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMAdd(pb_istream_t *stream) {
  memset(&_msg_pwm_add, 0, sizeof(_msg_pwm_add));
  return pb_decode(stream, wippersnapper_pwm_PWMAdd_fields, &_msg_pwm_add);
}

/*!
    @brief  Returns a pointer to the PWMAdd message.
    @return Pointer to the PWMAdd message.
*/
wippersnapper_pwm_PWMAdd *PWMModel::GetPWMAddMsg() { return &_msg_pwm_add; }

/*!
    @brief  Encodes a PWMAdded message with the given pin name and attach
   status.
    @param  pin_name  The name of the pin.
    @param  did_attach  True if the pin was successfully attached, false
   otherwise.
    @return true if successful, false otherwise.
*/
bool PWMModel::EncodePWMAdded(char *pin_name, bool did_attach) {
  // Fill the message
  memset(&_msg_pwm_added, 0, sizeof(_msg_pwm_added));
  _msg_pwm_added.did_attach = did_attach;
  strncpy(_msg_pwm_added.pin, pin_name, sizeof(_msg_pwm_added.pin));
  // Encode it!
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, wippersnapper_pwm_PWMAdded_fields,
                           &_msg_pwm_added))
    return false;
  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_pwm_PWMAdded_fields,
                   &_msg_pwm_added);
}

/*!
    @brief  Returns a pointer to the PWMAdded message.
    @return Pointer to the PWMAdded message.
*/
wippersnapper_pwm_PWMAdded *PWMModel::GetPWMAddedMsg() {
  return &_msg_pwm_added;
}

/*!
    @brief  Decodes a PWMRemove message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMRemove(pb_istream_t *stream) {
  memset(&_msg_pwm_remove, 0, sizeof(_msg_pwm_remove));
  return pb_decode(stream, wippersnapper_pwm_PWMRemove_fields,
                   &_msg_pwm_remove);
}

/*!
    @brief  Returns a pointer to the PWMRemove message.
    @return Pointer to the PWMRemove message.
*/
wippersnapper_pwm_PWMRemove *PWMModel::GetPWMRemoveMsg() {
  return &_msg_pwm_remove;
}

/*!
    @brief  Decodes a PWMWriteDutyCycle message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMWriteDutyCycle(pb_istream_t *stream) {
/*   memset(&_msg_pwm_write_duty_cycle, 0, sizeof(_msg_pwm_write_duty_cycle));
  return pb_decode(stream, wippersnapper_pwm_PWMWriteDutyCycle_fields,
                   &_msg_pwm_write_duty_cycle); */
                   return false; // TODO: Remove once re-implemented
}

/*!
    @brief  Returns a pointer to the PWMWriteDutyCycle message.
    @return Pointer to the PWMWriteDutyCycle message.
*/
/* wippersnapper_pwm_PWMWriteDutyCycle *PWMModel::GetPWMWriteDutyCycleMsg() {
  return &_msg_pwm_write_duty_cycle;
} */

/*!
    @brief  Decodes a PWMWriteFrequency message from an input stream.
    @param  stream  The stream to decode from.
    @return true if successful, false otherwise.
*/
bool PWMModel::DecodePWMWriteFrequency(pb_istream_t *stream) {
/*   memset(&_msg_pwm_write_frequency, 0, sizeof(_msg_pwm_write_frequency));
  return pb_decode(stream, wippersnapper_pwm_PWMWriteFrequency_fields,
                   &_msg_pwm_write_frequency); */
  return false;// TODO: Remove once re-implemented
}

/*!
    @brief  Returns a pointer to the PWMWriteFrequency message.
    @return Pointer to the PWMWriteFrequency message.
*/
/* wippersnapper_pwm_PWMWriteFrequency *PWMModel::GetPWMWriteFrequencyMsg() {
  return &_msg_pwm_write_frequency;
} */
