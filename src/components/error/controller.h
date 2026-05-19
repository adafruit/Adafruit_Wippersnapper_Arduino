/*!
 * @file src/components/error/controller.h
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
#ifndef WS_CONTROLLER_MODEL
#define WS_CONTROLLER_MODEL
#include "model.h"
#include "wippersnapper.h"

class ErrorModel;    ///< Forward declaration
class wippersnapper; ///< Forward declaration

/*!
    @brief  Provides an interface for routing and handling
            error.proto messages.
*/
class ErrorController {
public:
  ErrorController();
  ~ErrorController();
  bool Router(pb_istream_t *stream);
  bool publishComponentError(const char *pin, const char *error_msg);
  bool publishComponentError(ws_i2c_Descriptor i2c, const char *error_msg);
  bool publishComponentError(ws_uart_Descriptor uart, const char *error_msg);

private:
  bool HandleThrottle(const ws_error_ThrottleError &throttle);
  bool HandleBan(const ws_error_BanError &ban);
  ErrorModel *_model = nullptr; ///< Instance of ErrorModel class
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_CONTROLLER_MODEL