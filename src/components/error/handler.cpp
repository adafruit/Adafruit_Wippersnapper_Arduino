/*!
 * @file src/components/error/handler.cpp
 *
 * Handler for Wippersnapper error.proto API.
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
#include "handler.h"

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
  const char *str = (const char *)*arg;
  if (!str) {
    return pb_encode_string(stream, (const uint8_t *)"", 0);
  }

  size_t len = strlen(str);
  return pb_encode_string(stream, (const uint8_t *)str, len);
}

/*!
    @brief  ErrorHandler constructor
*/
ErrorHandler::ErrorHandler() { _d2b_msg = ws_error_D2B_init_zero; }

/*!
    @brief  ErrorHandler destructor
*/
ErrorHandler::~ErrorHandler() {}

/*!
    @brief  Routes messages using the error.proto API to the
            appropriate handler functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool ErrorHandler::Router(pb_istream_t *stream) {
  ws_error_B2D b2d = ws_error_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_error_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[error] ERROR: Unable to decode B2D envelope");
    return false;
  }

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
bool ErrorHandler::HandleBan(const ws_error_BanError &ban) {
  WS_DEBUG_PRINTLN("[ERROR] Received IO Ban from broker");
  if (!Ws._mqttV2->disconnect()) {
    WS_DEBUG_PRINTLN("ERROR: Unable to disconnect from MQTT broker!");
  }

  Ws.haltErrorV2("IO MQTT Ban Error");
  return true;
}

/*!
    @brief  Handles IO Throttle error messages from Adafruit IO.
    @param  throttle
            The throttle message from Adafruit IO.
    @return True if throttle was handled successfully, False otherwise.
*/
bool ErrorHandler::HandleThrottle(const ws_error_ThrottleError &throttle) {
  WS_DEBUG_PRINT("[ERROR] Device throttled for: ");
  WS_DEBUG_PRINTVAR(throttle.timeout);
  WS_DEBUG_PRINTLN(" seconds");
  WS_DEBUG_PRINTLN("[ERROR] Delaying command execution...");

  unsigned long duration_throttle = (unsigned long)throttle.timeout * 1000UL;
  unsigned long time_elapsed = 0UL;

  while (time_elapsed < duration_throttle) {
    unsigned long time_remaining = duration_throttle - time_elapsed;
    unsigned long time_delay = 0UL;
    if (time_remaining > WS_DEVICE_PING_MS) {
      time_delay = WS_DEVICE_PING_MS;
    } else {
      time_delay = time_remaining;
    }

    delay(time_delay);
    time_elapsed += time_delay;

    Ws._wdt->feed();
    Ws._mqttV2->ping();
  }

  WS_DEBUG_PRINTLN("[ERROR] Throttle period ended. Resuming normal operation.");
  return true;
}

/*!
    @brief  Publishes a component error for a pin-based component.
    @param  pin
            The pin name string.
    @param  error_msg
            The error message string.
    @return True if the error was published successfully, False otherwise.
*/
bool ErrorHandler::publishComponentError(const char *pin,
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

/*!
    @brief  Publishes a component error for an I2C-based component.
    @param  i2c
            The I2C descriptor.
    @param  error_msg
            The error message string.
    @return True if the error was published successfully, False otherwise.
*/
bool ErrorHandler::publishComponentError(ws_i2c_Descriptor i2c,
                                         const char *error_msg) {
  WS_DEBUG_PRINT("[error] I2C 0x");
  WS_DEBUG_PRINTHEX(i2c.address);
  WS_DEBUG_PRINT(": ");
  WS_DEBUG_PRINTLNVAR(error_msg);

  _d2b_msg = ws_error_D2B_init_zero;
  _d2b_msg.which_payload = ws_error_D2B_component_tag;

  ws_error_ComponentError *comp = &_d2b_msg.payload.component;
  comp->message.funcs.encode = encode_string_callback;
  comp->message.arg = (void *)error_msg;
  comp->which_descriptor = ws_error_ComponentError_i2c_tag;
  comp->descriptor.i2c = i2c;

  return publishD2B();
}

/*!
    @brief  Publishes a component error for an SPI-based component.
    @param  spi
            The SPI descriptor.
    @param  error_msg
            The error message string.
    @return True if the error was published successfully, False otherwise.
*/
bool ErrorHandler::publishComponentError(ws_spi_Descriptor spi,
                                         const char *error_msg) {
  WS_DEBUG_PRINT("[error] SPI CS pin ");
  WS_DEBUG_PRINTVAR(spi.pin_cs);
  WS_DEBUG_PRINT(": ");
  WS_DEBUG_PRINTLNVAR(error_msg);

  _d2b_msg = ws_error_D2B_init_zero;
  _d2b_msg.which_payload = ws_error_D2B_component_tag;

  ws_error_ComponentError *comp = &_d2b_msg.payload.component;
  comp->message.funcs.encode = encode_string_callback;
  comp->message.arg = (void *)error_msg;
  comp->which_descriptor = ws_error_ComponentError_spi_tag;
  comp->descriptor.spi = spi;

  return publishD2B();
}

/*!
    @brief  Publishes a component error for a UART-based component.
    @param  uart
            The UART descriptor.
    @param  error_msg
            The error message string.
    @return True if the error was published successfully, False otherwise.
*/
bool ErrorHandler::publishComponentError(ws_uart_Descriptor uart,
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

/*!
    @brief  Encodes and publishes the D2B error message.
    @return True if published successfully, False otherwise.
*/
bool ErrorHandler::publishD2B() {
  return Ws.PublishD2b(ws_signal_DeviceToBroker_error_tag, &_d2b_msg);
}
