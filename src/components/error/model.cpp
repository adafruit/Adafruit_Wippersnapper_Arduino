/*!
 * @file src/components/error/model.cpp
 *
 * Model for the Wippersnapper error.proto API.
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

// TODO: Remove all debug prints after we are confident during testing

/*!
    @brief  ErrorModel constructor
*/
ErrorModel::ErrorModel() {}

/*!
    @brief  ErrorModel destructor
*/
ErrorModel::~ErrorModel() {}

/*!
    @brief  Callback function to encode a string for nanopb.
    @param  stream
            The nanopb output stream.
    @param  field
            The nanopb field descriptor.
    @param  arg
            Pointer to the string to encode.
    @return True if encoding was successful, False otherwise.
*/
static bool encode_string_callback(pb_ostream_t *stream,
                                   const pb_field_t *field, void *const *arg) {
  // Retrieve the string from arg
  const char *str = (const char *)*arg;
  // Handle null string case
  if (!str) {
    return pb_encode_string(stream, (const uint8_t *)"", 0);
  }

  // Calculate string length and encode
  size_t len = strlen(str);
  return pb_encode_string(stream, (const uint8_t *)str, len);
}

bool ErrorModel::FillErrorD2B(pb_size_t which_component_type,
                              pb_size_t which_component_id, pb_callback_t pin,
                              pb_callback_t error_msg) {
  _error_d2b_msg = ws_error_ErrorD2B_init_zero;
  _error_d2b_msg.type = (ws_error_ComponentType)which_component_type;
  _error_d2b_msg.which_component_id = which_component_id;
  _error_d2b_msg.component_id.pin.funcs.encode = encode_string_callback;
  _error_d2b_msg.component_id.pin.arg = pin.arg;
  _error_d2b_msg.error = error_msg;
  return true;
}

ws_error_ErrorD2B *ErrorModel::getErrorD2BMessage() { return &_error_d2b_msg; }

bool ErrorModel::encodeErrorD2B(uint8_t *buffer, size_t buffer_size,
                                size_t *encoded_size) {
  // Get size of the encoded ErrorD2B message
  size_t sz_error_d2b_msg;
  if (!pb_get_encoded_size(&sz_error_d2b_msg, ws_error_ErrorD2B_fields,
                           &_error_d2b_msg)) {
    WS_DEBUG_PRINTLN("[Error] ERROR: Unable to get size of ErrorD2B message");
    return false;
  }

  // Encode the ErrorD2B message
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buffer, sz_error_d2b_msg);
  if (!pb_encode(&msg_stream, ws_error_ErrorD2B_fields, &_error_d2b_msg)) {
    WS_DEBUG_PRINTLN("[Error] ERROR: Unable to encode ErrorD2B message");
    return false;
  }
  *encoded_size = msg_stream.bytes_written;
  return true;
}