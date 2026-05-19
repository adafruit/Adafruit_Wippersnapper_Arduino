/*!
 * @file src/components/error/controller.cpp
 *
 * Controller for Wippersnapper error.proto API.
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
#include "controller.h"

/*!
    @brief  ErrorController constructor
*/
ErrorController::ErrorController() { _model = new ErrorModel(); }

/*!
    @brief  ErrorController destructor
*/
ErrorController::~ErrorController() { delete _model; }

/*!
    @brief  Routes messages using the error.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool ErrorController::Router(pb_istream_t *stream) {
  // Attempt to decode the Error B2D envelope
  ws_error_B2D b2d = ws_error_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_error_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[error] ERROR: Unable to decode B2D envelope");
    return false;
  }

  // Handle the message based on the tag type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_error_B2D_ban_tag:
    WS_DEBUG_PRINTLN("[error] Received BanError message from broker");
    res = HandleBan(b2d.payload.ban);
    break;
  case ws_error_B2D_throttle_tag:
    WS_DEBUG_PRINTLN("[error] Received ThrottleError message from broker");
    res = HandleThrottle(b2d.payload.throttle);
    break;
  default:
    WS_DEBUG_PRINTLN("[error] WARNING: Unhandled B2D message tag!");
    break;
  }
  return res;
}

/*!
    @brief  Handles IO Ban error messages from Adafruit IO.
    @param  ban
            The ban message from Adafruit IO.
    @return True if ban was handled successfully, False otherwise.
*/
bool ErrorController::HandleBan(const ws_error_BanError &ban) {
  WS_DEBUG_PRINTLN("[ERROR] Received IO Ban from broker");
  // Disconnect client from broker
  if (!Ws._mqttV2->disconnect()) {
    WS_DEBUG_PRINTLN("ERROR: Unable to disconnect from MQTT broker!");
  }

  // Let the device fall into halted state
  Ws.haltErrorV2("IO MQTT Ban Error");
  return true;
}

/*!
    @brief  Handles IO Throttle error messages from Adafruit IO.
    @param  throttle
            The throttle message from Adafruit IO.
    @return True if throttle was handled successfully, False otherwise.
*/
bool ErrorController::HandleThrottle(const ws_error_ThrottleError &throttle) {
  WS_DEBUG_PRINT("[ERROR] Device throttled for: ");
  WS_DEBUG_PRINTVAR(throttle.timeout);
  WS_DEBUG_PRINTLN(" seconds");
  WS_DEBUG_PRINTLN("[ERROR] Delaying command execution...");

  // Delay based on throttle timeout
  unsigned long duration_throttle = (unsigned long)throttle.timeout * 1000UL;
  unsigned long time_elapsed = 0UL;

  // Throttle loop
  while (time_elapsed < duration_throttle) {
    // Calculate remaining time
    unsigned long time_remaining = duration_throttle - time_elapsed;
    unsigned long time_delay = 0UL;
    if (time_remaining > WS_DEVICE_PING_MS) {
      time_delay = WS_DEVICE_PING_MS;
    } else {
      time_delay = time_remaining;
    }

    // Delay for time_delay milliseconds and update elapsed time
    delay(time_delay);
    time_elapsed += time_delay;

    // Feed the watchdog and ping MQTT to keep connection alive
    Ws._wdt->feed();
    Ws._mqttV2->ping();
  }

  WS_DEBUG_PRINTLN("[ERROR] Throttle period ended. Resuming normal operation.");
  return true;
}

bool ErrorController::publishComponentError(const char *pin,
                                             const char *error_msg) {
  return _model->publishComponentError(pin, error_msg);
}

bool ErrorController::publishComponentError(ws_i2c_Descriptor i2c,
                                             const char *error_msg) {
  // Print the eror out using its descriptor
  WS_DEBUG_PRINT("[error] I2C 0x");
  WS_DEBUG_PRINT(i2c.address, HEX);
  WS_DEBUG_PRINT(": ");
  WS_DEBUG_PRINTLN(error_msg);
  // Then, publish it
  return _model->publishComponentError(i2c, error_msg);
}

bool ErrorController::publishComponentError(ws_uart_Descriptor uart,
                                             const char *error_msg) {
  return _model->publishComponentError(uart, error_msg);
}
