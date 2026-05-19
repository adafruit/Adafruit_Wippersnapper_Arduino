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
    @brief  Provides an interface for creating, encoding, and publishing
            error.proto messages.
*/
class ErrorModel {
public:
  ErrorModel();
  ~ErrorModel();
  bool publishComponentError(const char *pin, const char *error_msg);
  bool publishComponentError(ws_i2c_Descriptor i2c, const char *error_msg);
  bool publishComponentError(ws_uart_Descriptor uart, const char *error_msg);
private:
  ws_error_D2B _d2b_msg; ///< D2B message instance
  bool publishD2B();
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_ERROR_MODEL_H