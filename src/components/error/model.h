/*!
 * @file src/components/error/model.h
 *
 * Model for the Wippersnapper error proto API.
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
#ifndef WS_ERROR_MODEL_H
#define WS_ERROR_MODEL_H
#include "wippersnapper.h"

class wippersnapper; ///< Forward declaration

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from error.proto.
*/
bool encode_string_callback(pb_ostream_t *stream, const pb_field_t *field,
                            void *const *arg);

class ErrorModel {
public:
  ErrorModel();
  ~ErrorModel();
  bool FillErrorD2B(pb_size_t which_component_type,
                    pb_size_t which_component_id, pb_callback_t pin,
                    pb_callback_t error_msg);
  ws_error_ErrorD2B *getErrorD2BMessage();
  bool encodeErrorD2B(uint8_t *buffer, size_t buffer_size,
                      size_t *encoded_size);

private:
  ws_error_ErrorD2B _error_d2b_msg; ///< ErrorD2B message instance;
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_ERROR_MODEL_H