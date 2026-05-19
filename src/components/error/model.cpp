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
                                   const pb_field_t *field,
                                   void *const *arg) {
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

bool ErrorModel::publishComponentError(const char *pin,
                                       const char *error_msg) {
  _d2b_msg = ws_error_D2B_init_zero;
  _d2b_msg.which_payload = ws_error_D2B_component_tag;

  ws_error_ComponentError *comp = &_d2b_msg.payload.component;
  comp->message.funcs.encode = encode_string_callback;
  comp->message.arg = (void *)error_msg;
  comp->which_descriptor = ws_error_ComponentError_pin_tag;
  comp->descriptor.pin.funcs.encode = encode_string_callback;
  comp->descriptor.pin.arg = (void *)pin;

  return publishD2B();
}

bool ErrorModel::publishComponentError(ws_i2c_Descriptor i2c,
                                       const char *error_msg) {
  _d2b_msg = ws_error_D2B_init_zero;
  _d2b_msg.which_payload = ws_error_D2B_component_tag;

  ws_error_ComponentError *comp = &_d2b_msg.payload.component;
  comp->message.funcs.encode = encode_string_callback;
  comp->message.arg = (void *)error_msg;
  comp->which_descriptor = ws_error_ComponentError_i2c_tag;
  comp->descriptor.i2c = i2c;

  return publishD2B();
}

bool ErrorModel::publishComponentError(ws_uart_Descriptor uart,
                                       const char *error_msg) {
  _d2b_msg = ws_error_D2B_init_zero;
  _d2b_msg.which_payload = ws_error_D2B_component_tag;

  ws_error_ComponentError *comp = &_d2b_msg.payload.component;
  comp->message.funcs.encode = encode_string_callback;
  comp->message.arg = (void *)error_msg;
  comp->which_descriptor = ws_error_ComponentError_uart_tag;
  comp->descriptor.uart = uart;

  return publishD2B();
}

bool ErrorModel::publishD2B() {
  return Ws.PublishD2b(ws_signal_DeviceToBroker_error_tag, &_d2b_msg);
}