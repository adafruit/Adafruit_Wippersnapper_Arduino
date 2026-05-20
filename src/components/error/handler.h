/*!
 * @file src/components/error/handler.h
 *
 * Handler for the Wippersnapper error proto API.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_ERROR_HANDLER_H
#define WS_ERROR_HANDLER_H
#include "wippersnapper.h"

class wippersnapper; ///< Forward declaration

/*!
    @brief  Provides an interface for routing, handling, encoding,
            and publishing error.proto messages.
*/
class ErrorHandler {
public:
  ErrorHandler();
  ~ErrorHandler();
  bool Router(pb_istream_t *stream);
  bool publishComponentError(const char *pin, const char *error_msg);
  bool publishComponentError(ws_i2c_Descriptor i2c, const char *error_msg);
  bool publishComponentError(ws_uart_Descriptor uart, const char *error_msg);

private:
  bool HandleThrottle(const ws_error_ThrottleError &throttle);
  bool HandleBan(const ws_error_BanError &ban);
  bool publishD2B();
  ws_error_D2B _d2b_msg; ///< D2B message instance
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_ERROR_HANDLER_H
