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
  ws_error_ErrorB2D b2d = ws_error_ErrorB2D_init_zero;
  if (!ws_pb_decode(stream, ws_error_ErrorB2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[error] ERROR: Unable to decode ErrorB2D envelope");
    return false;
  }

  // Handle the message based on the tag type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_error_ErrorB2D_io_ban_tag:
    WS_DEBUG_PRINTLN("[error] Received ErrorIOBan message from broker");
    res = HandleBan(b2d.payload.io_ban);
    break;
  case ws_error_ErrorB2D_io_throttle_tag:
    WS_DEBUG_PRINTLN("[error] Received ErrorIOThrottle message from broker");
    res = HandleThrottle(b2d.payload.io_throttle);
    break;
  default:
    WS_DEBUG_PRINTLN("[error] WARNING: Unhandled ErrorB2D message tag!");
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
bool ErrorController::HandleBan(const ws_error_ErrorIOBan &ban) {
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
bool ErrorController::HandleThrottle(const ws_error_ErrorIOThrottle &throttle) {
  WS_DEBUG_PRINT("[ERROR] Device throttled for: ");
  WS_DEBUG_PRINT(throttle.timeout);
  WS_DEBUG_PRINTLN("seconds");
  WS_DEBUG_PRINTLN("[ERROR] Delaying command execution...");

  // Delay based on throttle timeout
  unsigned long duration_throttle = (unsigned long)throttle.timeout * 1000UL;
  unsigned long time_elapsed = 0UL;

  // Throttle loop
  while (time_elapsed < duration_throttle) {
    // Calculate remaining time
    unsigned long time_remaining = duration_throttle - time_elapsed;
    unsigned long time_delay = 0UL;
    if (time_remaining > WS_KEEPALIVE_INTERVAL_MS) {
      time_delay = WS_KEEPALIVE_INTERVAL_MS;
    } else {
      time_delay = time_remaining;
    }

    // Delay for time_delay milliseconds and update elapsed time
    delay(time_delay);
    time_elapsed += time_delay;

    // Feed the watchdog and ping MQTT to keep connection alive
    Ws.feedWDTV2();
    Ws._mqttV2->ping();
  }

  WS_DEBUG_PRINTLN("[ERROR] Throttle period ended. Resuming normal operation.");
  return true;
}

bool ErrorController::PublishError(pb_size_t which_component_type,
                                   pb_size_t which_component_id,
                                   pb_callback_t pin, pb_callback_t error_msg) {
  ws_error_ErrorD2B error_d2b = ws_error_ErrorD2B_init_zero;
  // Fill the ErrorD2B message
  if (!_model->FillErrorD2B(which_component_type, which_component_id, pin,
                            error_msg)) {
    WS_DEBUG_PRINTLN("[Error] ERROR: Unable to fill ErrorD2B message");
    return false;
  }

  // Get the size needed for encoding
  size_t encoded_size;
  if (!pb_get_encoded_size(&encoded_size, ws_error_ErrorD2B_fields,
                           _model->getErrorD2BMessage())) {
    WS_DEBUG_PRINTLN("[Error] ERROR: Unable to get size of ErrorD2B message");
    return false;
  }

  // Validate buffer size won't exceed a reasonable limit for a ErrorD2B message
  if (encoded_size == 0 || encoded_size > 1024) {
    WS_DEBUG_PRINTLN(
        "[Error] ERROR: Invalid encoded size for ErrorD2B message");
    return false;
  }

  // Attempt to allocate buffer and encode the message
  uint8_t buf[encoded_size];
  if (!_model->encodeErrorD2B(buf, encoded_size, &encoded_size)) {
    WS_DEBUG_PRINTLN("[Error] ERROR: Unable to encode ErrorD2B message");
    return false;
  }

  // Publish the ErrorD2B message to the broker
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_error_tag, (void *)buf)) {
    WS_DEBUG_PRINTLN("[Error] ERROR: Unable to publish ErrorD2B message");
    return false;
  }

  return true;
}

// bool PublishError(pb_size_t which_component_type, pb_size_t
// which_component_id, ws_i2c_DeviceDescriptor i2c, pb_callback_t error_msg);
// bool PublishError(pb_size_t which_component_type, pb_size_t
// which_component_id, ws_uart_Descriptor uart, pb_callback_t error_msg);

/*! Helper Functions */

/*!
    @brief  Publishes an AnalogIO error message to the broker.
    @param  error_msg
            The error message to publish.
    @param  pin_name
            The name of the pin that caused the error.
    @return True if the message was successfully published, False otherwise.
*/
bool ErrorController::PublishAnalogIO(const char *error_msg,
                                      const char *pin_name) {
  if (!error_msg || !pin_name)
    return false;

  // Create callbacks for pin name and error message
  pb_callback_t error_msg_callback;
  error_msg_callback.funcs.encode = encode_string_callback;
  error_msg_callback.arg = (void *)error_msg;

  pb_callback_t pin_callback;
  pin_callback.funcs.encode = encode_string_callback;
  pin_callback.arg = (void *)pin_name;

  // Pass to PublishError
  return PublishError(ws_signal_DeviceToBroker_analogio_tag,
                      ws_error_ErrorD2B_pin_tag, pin_callback,
                      error_msg_callback);
}

/*!
    @brief  Publishes a DigitalIO error message to the broker.
    @param  error_msg
            The error message to publish.
    @param  pin_name
            The name of the pin that caused the error.
    @return True if the message was successfully published, False otherwise.
*/
bool ErrorController::PublishDigitalIO(const char *error_msg,
                                       const char *pin_name) {
  if (!error_msg || !pin_name)
    return false;

  pb_callback_t error_msg_callback;
  error_msg_callback.funcs.encode = encode_string_callback;
  error_msg_callback.arg = (void *)error_msg;

  pb_callback_t pin_callback;
  pin_callback.funcs.encode = encode_string_callback;
  pin_callback.arg = (void *)pin_name;

  return PublishError(ws_signal_DeviceToBroker_digitalio_tag,
                      ws_error_ErrorD2B_pin_tag, pin_callback,
                      error_msg_callback);
}

/*!
    @brief  Publishes a DS18x20 error message to the broker.
    @param  error_msg
            The error message to publish.
    @param  pin_name
            The name of the pin that caused the error.
    @return True if the message was successfully published, False otherwise.
*/
bool ErrorController::PublishDS18x20(const char *error_msg,
                                     const char *pin_name) {
  if (!error_msg || !pin_name)
    return false;

  pb_callback_t error_msg_callback;
  error_msg_callback.funcs.encode = encode_string_callback;
  error_msg_callback.arg = (void *)error_msg;

  pb_callback_t pin_callback;
  pin_callback.funcs.encode = encode_string_callback;
  pin_callback.arg = (void *)pin_name;

  return PublishError(ws_signal_DeviceToBroker_ds18x20_tag,
                      ws_error_ErrorD2B_pin_tag, pin_callback,
                      error_msg_callback);
}

/*!
    @brief  Publishes a Pixels error message to the broker.
    @param  error_msg
            The error message to publish.
    @param  pin_name
            The name of the pin that caused the error.
    @return True if the message was successfully published, False otherwise.
*/
bool ErrorController::PublishPixels(const char *error_msg,
                                    const char *pin_name) {
  if (!error_msg || !pin_name)
    return false;

  pb_callback_t error_msg_callback;
  error_msg_callback.funcs.encode = encode_string_callback;
  error_msg_callback.arg = (void *)error_msg;

  pb_callback_t pin_callback;
  pin_callback.funcs.encode = encode_string_callback;
  pin_callback.arg = (void *)pin_name;

  return PublishError(ws_signal_DeviceToBroker_pixels_tag,
                      ws_error_ErrorD2B_pin_tag, pin_callback,
                      error_msg_callback);
}

/*!
    @brief  Publishes a PWM error message to the broker.
    @param  error_msg
            The error message to publish.
    @param  pin_name
            The name of the pin that caused the error.
    @return True if the message was successfully published, False otherwise.
*/
bool ErrorController::PublishPWM(const char *error_msg, const char *pin_name) {
  if (!error_msg || !pin_name)
    return false;

  pb_callback_t error_msg_callback;
  error_msg_callback.funcs.encode = encode_string_callback;
  error_msg_callback.arg = (void *)error_msg;

  pb_callback_t pin_callback;
  pin_callback.funcs.encode = encode_string_callback;
  pin_callback.arg = (void *)pin_name;

  return PublishError(ws_signal_DeviceToBroker_pwm_tag,
                      ws_error_ErrorD2B_pin_tag, pin_callback,
                      error_msg_callback);
}

/*!
    @brief  Publishes a Servo error message to the broker.
    @param  error_msg
            The error message to publish.
    @param  pin_name
            The name of the pin that caused the error.
    @return True if the message was successfully published, False otherwise.
*/
bool ErrorController::PublishServo(const char *error_msg,
                                   const char *pin_name) {
  if (!error_msg || !pin_name)
    return false;

  pb_callback_t error_msg_callback;
  error_msg_callback.funcs.encode = encode_string_callback;
  error_msg_callback.arg = (void *)error_msg;

  pb_callback_t pin_callback;
  pin_callback.funcs.encode = encode_string_callback;
  pin_callback.arg = (void *)pin_name;

  return PublishError(ws_signal_DeviceToBroker_servo_tag,
                      ws_error_ErrorD2B_pin_tag, pin_callback,
                      error_msg_callback);
}
