/*!
 * @file src/components/sleep/model.cpp
 *
 * Model for the sleep.proto message.
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
#ifdef ARDUINO_ARCH_ESP32
#include "model.h"

/*!
    @brief  SleepModel constructor
*/
SleepModel::SleepModel() {
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));
  memset(&_msg_sleep_wake, 0, sizeof(_msg_sleep_wake));
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));
}

/*!
    @brief  SleepModel destructor
*/
SleepModel::~SleepModel() {
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));
  memset(&_msg_sleep_wake, 0, sizeof(_msg_sleep_wake));
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));
}

/*!
    @brief  Gets a Sleep Enter message object.
    @return Sleep Enter message object.
*/
ws_sleep_Enter *SleepModel::GetSleepEnterMsg() { return &_msg_sleep_enter; }

/*!
    @brief  Gets a Sleep Goodnight message object.
    @return Sleep Goodnight message object.
*/
ws_sleep_Goodnight *SleepModel::GetSleepGoodnightMsg() {
  return &_msg_sleep_goodnight;
}

/*!
    @brief  Gets a Sleep Wake message object.
    @return Sleep Wake message object.
*/
ws_sleep_Wake *SleepModel::GetSleepWakeMsg() { return &_msg_sleep_wake; }

/*!
    @brief  Decodes a Sleep Enter message into the _msg_sleep_enter
            object from a nanopb stream.
    @param  stream
            The nanopb input stream.
    @return True if the Sleep Enter message was successfully decoded.
*/
bool SleepModel::DecodeSleepEnter(pb_istream_t *stream) {
  // Zero-out the Sleep Enter message struct to ensure a clean decode
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));

  // Decode the stream into a Sleep Enter message
  return pb_decode(stream, ws_sleep_Enter_fields, &_msg_sleep_enter);
}

/*!
    @brief Encodes a C-string for the Goodnight.message callback field
    @param stream
           The nanopb output stream.
    @param field
           The nanopb field descriptor.
    @param arg
           Pointer to the C-string to encode.
*/
static bool _encode_goodnight_message(pb_ostream_t *stream,
                                      const pb_field_t *field,
                                      void *const *arg) {
  const char *msg = (const char *)(*arg);
  if (!pb_encode_tag_for_field(stream, field))
    return false;
  return pb_encode_string(stream, (const uint8_t *)msg,
                          (size_t)strlen(msg));
}

/*!
    @brief  Encodes a Sleep Goodnight message into the _msg_sleep_goodnight
            object.
    @param  msg
            A short, human-readable message indicating the device is
            going to sleep.
    @return True if the Sleep Goodnight message was successfully encoded.
            False if encoding resulted in a failure.
*/
bool SleepModel::EncodeSleepGoodnight(const char *msg) {
  // Initialize the Goodnight message
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));

  // Set up callback encoder for the string field
  _msg_sleep_goodnight.message.arg = (void *)msg;
  _msg_sleep_goodnight.message.funcs.encode = _encode_goodnight_message;

  // Compute encoded size
  size_t sz_goodnight_msg;
  if (!pb_get_encoded_size(&sz_goodnight_msg, ws_sleep_Goodnight_fields,
                           &_msg_sleep_goodnight))
    return false;

  // Create an output stream and encode
  uint8_t buf[sz_goodnight_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_sleep_Goodnight_fields,
                   &_msg_sleep_goodnight);
}

/*!
    @brief  Encodes a Sleep Wake message into the _msg_sleep_wake object.
    @param  cause
            Wake cause for ESP32x MCUs.
    @param  sleep_duration
            Calculated duration of the sleep period, in seconds.
    @return True if the Sleep Wake message was successfully encoded.
            False if encoding resulted in a failure.
*/
bool SleepModel::EncodeSleepWake(ws_sleep_EspWakeCause cause,
                                 uint32_t sleep_duration) {
  // Initialize the Wake message
  memset(&_msg_sleep_wake, 0, sizeof(_msg_sleep_wake));

  // Fill the Wake message
  _msg_sleep_wake.which_WakeCause = ws_sleep_Wake_esp_tag;
  _msg_sleep_wake.WakeCause.esp = cause;
  _msg_sleep_wake.sleep_duration = sleep_duration;

  // Compute encoded size
  size_t sz_wake_msg;
  if (!pb_get_encoded_size(&sz_wake_msg, ws_sleep_Wake_fields,
                           &_msg_sleep_wake))
    return false;

  // Create an output stream and encode
  uint8_t buf[sz_wake_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_sleep_Wake_fields, &_msg_sleep_wake);
}

#endif // ARDUINO_ARCH_ESP32