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
#include "Wippersnapper_V2.h"
#include "model.h"

class ErrorModel;       ///< Forward declaration
class Wippersnapper_V2; ///< Forward declaration

/*!
    @brief  Provides an interface for routing and handling
            error.proto messages.
*/
class ErrorController {
public:
  ErrorController();
  ~ErrorController();
  bool Router(pb_istream_t *stream);

private:
  bool HandleThrottle(const ws_error_ErrorIOThrottle &throttle);
  bool HandleBan(const ws_error_ErrorIOBan &ban);
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_CONTROLLER_MODEL