/*!
 * @file src/components/checkin/model.cpp
 *
 * Model for the Wippersnapper checkin proto API.
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
#if defined(ARDUINO_ARCH_ESP32)
#include "mbedtls/sha256.h"
#elif defined(ARDUINO_ARCH_RP2350)
#include <bearssl/bearssl_hash.h>
#endif

// SHA-256 of the last fully handled checkin response. Plain RAM on purpose:
// it survives light sleep, but is cleared by a reset or deep sleep wake, where
// nothing is configured yet and the response must always be applied.
// All zero until a response hash is saved.
static uint8_t last_sha_checkin_response[CHECKIN_SHA_LEN];
// Components applied from checkin responses, same RAM lifetime as above
static std::vector<CheckinComponent> checkin_components;

/*!
    @brief  Computes the SHA-256 of a buffer.
    @param  data
            Buffer to hash.
    @param  len
            Length of the buffer.
    @param  out
            CHECKIN_SHA_LEN byte digest output.
    @returns True if hashed, False if hashing is unsupported or failed.
*/
static bool sha256(const uint8_t *data, size_t len, uint8_t *out) {
#if defined(ARDUINO_ARCH_ESP32)
  return mbedtls_sha256(data, len, out, 0) == 0;
#elif defined(ARDUINO_ARCH_RP2350)
  br_sha256_context ctx;
  br_sha256_init(&ctx);
  br_sha256_update(&ctx, data, len);
  br_sha256_out(&ctx, out);
  return true;
#else
  (void)data;
  (void)len;
  (void)out;
  return false;
#endif
}

/*!
    @brief  Finds a component in the checkin component list.
    @param  type
            BrokerToDevice payload tag of the component.
    @param  remove
            Encoded Remove message identifying the component.
    @returns Index of the component, or -1 if not found.
*/
static int findComponent(pb_size_t type, const std::vector<uint8_t> &remove) {
  for (size_t i = 0; i < checkin_components.size(); i++) {
    if (checkin_components[i].type == type &&
        checkin_components[i].remove == remove)
      return (int)i;
  }
  return -1;
}

/*!
    @brief  Decodes a component's stored Remove message.
    @param  component
            The component to remove.
    @param  fields
            nanopb fields of the Remove message.
    @param  msg
            Zero-initialised Remove message to decode into.
    @returns True if decoded, False otherwise.
*/
static bool decodeRemove(const CheckinComponent &component,
                         const pb_msgdesc_t *fields, void *msg) {
  pb_istream_t stream =
      pb_istream_from_buffer(component.remove.data(), component.remove.size());
  return pb_decode(&stream, fields, msg);
}

// TODO: Remove all debug prints after we are confident during testing

/*!
    @brief  CheckinModel constructor
*/
CheckinModel::CheckinModel() {
  _got_response = false;
  memset(_sha_checkin_response, 0, CHECKIN_SHA_LEN);
}

/*!
    @brief  CheckinModel destructor
*/
CheckinModel::~CheckinModel() { _got_response = false; }

/*!
    @brief  Creates a ws_checkin_Request message and publishes
            it to the broker.
    @param  hardware_uid
            Hardware's unique identifier.
    @param  firmware_version
            WipperSnapper firmware version.
    @returns True if checkin was successful, False otherwise.
*/
bool CheckinModel::Checkin(const char *hardware_uid,
                           const char *firmware_version) {
  // Zero-out the D2B wrapper message
  memset(&_CheckinD2B, 0, sizeof(_CheckinD2B));

  // Create the CheckinRequest message
  _CheckinD2B.which_payload = ws_checkin_D2B_request_tag;
  strncpy(_CheckinD2B.payload.request.hardware_uid, hardware_uid,
          sizeof(_CheckinD2B.payload.request.hardware_uid) - 1);
  strncpy(_CheckinD2B.payload.request.firmware_version, firmware_version,
          sizeof(_CheckinD2B.payload.request.firmware_version) - 1);

  // Encode the D2B wrapper message
  size_t CheckinD2BSz;
  if (!pb_get_encoded_size(&CheckinD2BSz, ws_checkin_D2B_fields,
                           &_CheckinD2B)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to get encoded size of CheckinD2B message!");
    return false;
  }

  uint8_t buf[CheckinD2BSz];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&msg_stream, ws_checkin_D2B_fields, &_CheckinD2B)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Unable to encode CheckinD2B message!");
    return false;
  }

  // Publish out
  WS_DEBUG_PRINT("[checkin] Publishing CheckinRequest");
  if (!Ws->PublishD2b(ws_signal_DeviceToBroker_checkin_tag, &_CheckinD2B)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to publish CheckinRequest message!");
    return false;
  }
  WS_DEBUG_PRINTLN("...Published!");
  return true;
}

/*!
    @brief  Decodes a ws_checkin_Response message from the broker, without
            executing any component adds. No component_adds callbacks are
            set, so nanopb skips them. Component adds are executed later by
            ProcessComponentAdds(), after the controllers are configured.
    @param  stream
            Incoming data stream from buffer.
            Publishes a DeviceError to the broker on failure.
    @returns True if decoded successfully and the payload is a Response,
             False otherwise.
*/
bool CheckinModel::ProcessResponse(pb_istream_t *stream) {
  // Zero-out the B2D wrapper message
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));

  // Decode the B2D wrapper message
  if (!pb_decode(stream, ws_checkin_B2D_fields, &_CheckinB2D)) {
    char decode_error[128];
    snprintf(decode_error, sizeof(decode_error),
             "[checkin] ERROR: Unable to decode CheckinB2D message: %s",
             PB_GET_ERROR(stream));
    WS_DEBUG_PRINTLNVAR(decode_error);
    Ws->error_handler->publishDeviceError(decode_error);
    return false;
  }

  // Validate the response message type
  if (_CheckinB2D.which_payload != ws_checkin_B2D_response_tag) {
    const char *payload_error =
        "[checkin] ERROR: CheckinB2D message from broker is not a Response!";
    WS_DEBUG_PRINTLNVAR(payload_error);
    Ws->error_handler->publishDeviceError(payload_error);
    return false;
  }

  return true;
}

/*!
    @brief  Decodes the checkin response a second time, executing each
            component add. Must be called with a copy of the stream passed
            to ProcessResponse(), after the controllers are configured.
            A failed add is counted and skipped, it never fails the decode.
    @param  stream
            Copy of the incoming data stream, positioned at the response.
    @returns True if the response was decoded, False on a framing error.
*/
bool CheckinModel::ProcessComponentAdds(pb_istream_t *stream) {
  _adds_total = 0;
  _adds_failed = 0;

  // Re-zero the B2D wrapper, the response fields are re-decoded with the
  // same values so the getters remain valid after this pass
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));

  // Set up the pre-decode callback (cb_payload) which will be called BEFORE
  // the Response submessage is decoded. This allows us to set up the
  // component_adds callback before nanopb initializes the Response struct.
  _CheckinB2D.cb_payload.funcs.decode = &cbSetupResponse;
  _CheckinB2D.cb_payload.arg = this;

  if (!pb_decode(stream, ws_checkin_B2D_fields, &_CheckinB2D)) {
    WS_DEBUG_PRINT("[checkin] ERROR: Unable to decode component adds: ");
    WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(stream));
    return false;
  }
  return true;
}

/*!
    @brief  Returns the number of component adds seen during checkin.
    @returns Total component adds.
*/
uint16_t CheckinModel::GetAddsTotal() { return _adds_total; }

/*!
    @brief  Returns the number of component adds that failed during checkin.
    @returns Failed component adds.
*/
uint16_t CheckinModel::GetAddsFailed() { return _adds_failed; }

/*!
    @brief  Skips a component add whose submessage failed to decode, so the
            remaining adds can still be decoded. Logs only, the failure is
            not published to IO.
    @param  stream
            The add's substream, partially consumed by the failed decode.
    @param  add_type
            Name of the add type, for the log message.
    @returns True, so nanopb continues decoding the next add.
*/
bool CheckinModel::SkipFailedAdd(pb_istream_t *stream, const char *add_type) {
  WS_DEBUG_PRINT("[checkin] ERROR: Failed to decode ");
  WS_DEBUG_PRINTVAR(add_type);
  WS_DEBUG_PRINT(" add: ");
  WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(stream));
  // Drain what's left of this add, otherwise nanopb calls the callback
  // again on the leftover bytes
  if (stream->bytes_left > 0) {
    pb_read(stream, NULL, stream->bytes_left);
  }
  _adds_failed++;
  return true;
}

/*!
    @brief  Validates the decoded ws_checkin_Response from the broker.
            Must be called after ProcessResponse() returns true. If the
            response is invalid, publishes a DeviceError to the broker.
    @returns True if the response is R_OK, False otherwise.
*/
bool CheckinModel::ValidateResponse() {
  _got_response = false;
  const char *checkin_error = nullptr;
  switch (_CheckinB2D.payload.response.response) {
  case ws_checkin_Response_Response_R_OK:
    // Set flag to indicate a valid response was received
    _got_response = true;
    return true;
  case ws_checkin_Response_Response_R_BOARD_NOT_FOUND:
    checkin_error = "[checkin] ERROR: Board not found in definition index";
    break;
  case ws_checkin_Response_Response_R_UNSPECIFIED:
    checkin_error = "[checkin] ERROR: Unspecified checkin response from broker";
    break;
  default:
    checkin_error = "[checkin] ERROR: Unknown checkin response from broker";
    break;
  }
  WS_DEBUG_PRINTLNVAR(checkin_error);
  Ws->error_handler->publishDeviceError(checkin_error);
  return false;
}

/*!
    @brief  Configures controllers limits based on board definition
*/
void CheckinModel::ConfigureControllers() {
  Ws->digital_controller->SetMaxDigitalPins(GetTotalGPIOPins());
  Ws->analog_controller->SetMaxAnalogPins(GetTotalAnalogPins());
  Ws->analog_controller->SetRefVoltage(GetReferenceVoltage());
}

/*!
    @brief    Pre-decode callback for setting up component_adds callbacks.
              Called by nanopb via MSG_W_CB before the Response submessage
              is decoded. This allows us to set up nested callbacks before
              nanopb initializes the struct.
    @param    stream
              Incoming data stream (Response submessage bytes).
    @param    field
              Field descriptor for the Response field.
    @param    arg
              Pointer to CheckinModel instance.
    @returns  True to continue decoding (don't consume stream).
*/
bool CheckinModel::cbSetupResponse(pb_istream_t *stream,
                                   const pb_field_t *field, void **arg) {
  // Get the Response struct that nanopb will decode into
  ws_checkin_Response *response = (ws_checkin_Response *)field->pData;
  CheckinModel *model = (CheckinModel *)*arg;

  // Set up callbacks for ALL component types
  response->component_adds.digital_adds.funcs.decode = &cbDigitalAdds;
  response->component_adds.digital_adds.arg = model;

  response->component_adds.analog_adds.funcs.decode = &cbAnalogAdds;
  response->component_adds.analog_adds.arg = model;

  response->component_adds.servo_adds.funcs.decode = &cbServoAdds;
  response->component_adds.servo_adds.arg = model;

  response->component_adds.pwm_adds.funcs.decode = &cbPWMAdds;
  response->component_adds.pwm_adds.arg = model;

  response->component_adds.pixels_adds.funcs.decode = &cbPixelsAdds;
  response->component_adds.pixels_adds.arg = model;

  response->component_adds.ds18x20_adds.funcs.decode = &cbDs18x20Adds;
  response->component_adds.ds18x20_adds.arg = model;

  response->component_adds.uart_adds.funcs.decode = &cbUartAdds;
  response->component_adds.uart_adds.arg = model;

  response->component_adds.i2c_adds.funcs.decode = &cbI2cAdds;
  response->component_adds.i2c_adds.arg = model;

  response->component_adds.telemetry_adds.funcs.decode = &cbTelemetryAdds;
  response->component_adds.telemetry_adds.arg = model;

  response->component_adds.display_adds.funcs.decode = &cbDisplayAdds;
  response->component_adds.display_adds.arg = model;

  response->component_adds.expander_adds.funcs.decode = &cbExpanderAdds;
  response->component_adds.expander_adds.arg = model;

  // Return true WITHOUT consuming the stream - nanopb will continue
  // to decode the Response submessage with our callback now set up
  return true;
}

/*!
    @brief    Callback for decoding Digital Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbDigitalAdds(pb_istream_t *stream, const pb_field_t *field,
                                 void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_digital_Add add_msg = ws_digital_Add_init_zero;
  if (!pb_decode(stream, ws_digital_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "digital");
  }
  ws_digital_Remove remove_msg = ws_digital_Remove_init_zero;
  strncpy(remove_msg.pin, add_msg.pin, sizeof(remove_msg.pin) - 1);
  // Digital Add replaces an existing pin itself
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_digital_tag,
                          ws_digital_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (!model->CommitAdd(Ws->digital_controller->Handle_Digital_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding Analog Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbAnalogAdds(pb_istream_t *stream, const pb_field_t *field,
                                void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_analog_Add add_msg = ws_analog_Add_init_zero;
  if (!pb_decode(stream, ws_analog_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "analog");
  }
  ws_analog_Remove remove_msg = ws_analog_Remove_init_zero;
  strncpy(remove_msg.pin, add_msg.pin, sizeof(remove_msg.pin) - 1);
  // Analog Add replaces an existing pin itself
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_analog_tag,
                          ws_analog_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (!model->CommitAdd(Ws->analog_controller->Handle_Analog_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding Servo Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbServoAdds(pb_istream_t *stream, const pb_field_t *field,
                               void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_servo_Add add_msg = ws_servo_Add_init_zero;
  if (!pb_decode(stream, ws_servo_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "servo");
  }
  ws_servo_Remove remove_msg = ws_servo_Remove_init_zero;
  strncpy(remove_msg.servo_pin, add_msg.servo_pin,
          sizeof(remove_msg.servo_pin) - 1);
  // Servo Add does not replace an existing servo, remove it first
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_servo_tag,
                          ws_servo_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (action == CHECKIN_REPLACE &&
      !Ws->_servo_controller->Handle_Servo_Remove(&remove_msg))
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to remove component to replace");
  if (!model->CommitAdd(Ws->_servo_controller->Handle_Servo_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding PWM Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbPWMAdds(pb_istream_t *stream, const pb_field_t *field,
                             void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_pwm_Add add_msg = ws_pwm_Add_init_zero;
  if (!pb_decode(stream, ws_pwm_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "pwm");
  }
  ws_pwm_Remove remove_msg = ws_pwm_Remove_init_zero;
  strncpy(remove_msg.pin, add_msg.pin, sizeof(remove_msg.pin) - 1);
  // PWM Add replaces an existing pin itself
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_pwm_tag,
                          ws_pwm_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (!model->CommitAdd(Ws->_pwm_controller->Handle_PWM_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding Pixels Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbPixelsAdds(pb_istream_t *stream, const pb_field_t *field,
                                void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_pixels_Add add_msg = ws_pixels_Add_init_zero;
  if (!pb_decode(stream, ws_pixels_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "pixels");
  }
  ws_pixels_Remove remove_msg = ws_pixels_Remove_init_zero;
  strncpy(remove_msg.pin_data, add_msg.pin_data,
          sizeof(remove_msg.pin_data) - 1);
  // Pixels Add does not replace an existing strand, remove it first
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_pixels_tag,
                          ws_pixels_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (action == CHECKIN_REPLACE &&
      !Ws->_pixels_controller->Handle_Pixels_Remove(&remove_msg))
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to remove component to replace");
  if (!model->CommitAdd(Ws->_pixels_controller->Handle_Pixels_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding DS18x20 Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbDs18x20Adds(pb_istream_t *stream, const pb_field_t *field,
                                 void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_ds18x20_Add add_msg = ws_ds18x20_Add_init_zero;
  if (!pb_decode(stream, ws_ds18x20_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "ds18x20");
  }
  ws_ds18x20_Remove remove_msg = ws_ds18x20_Remove_init_zero;
  strncpy(remove_msg.onewire_pin, add_msg.onewire_pin,
          sizeof(remove_msg.onewire_pin) - 1);
  // DS18x20 Add does not replace an existing sensor, remove it first
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_ds18x20_tag,
                          ws_ds18x20_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (action == CHECKIN_REPLACE &&
      !Ws->_ds18x20_controller->Handle_Ds18x20Remove(&remove_msg))
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to remove component to replace");
  if (!model->CommitAdd(Ws->_ds18x20_controller->Handle_Ds18x20Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding UART Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbUartAdds(pb_istream_t *stream, const pb_field_t *field,
                              void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_uart_Add add_msg = ws_uart_Add_init_zero;
  if (!pb_decode(stream, ws_uart_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "uart");
  }
  ws_uart_Remove remove_msg = ws_uart_Remove_init_zero;
  remove_msg.has_descriptor = add_msg.has_descriptor;
  remove_msg.descriptor = add_msg.descriptor;
  // UART Add rejects a port already in use, remove it first
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_uart_tag,
                          ws_uart_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (action == CHECKIN_REPLACE &&
      !Ws->_uart_controller->Handle_UartRemove(&remove_msg))
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to remove component to replace");
  if (!model->CommitAdd(Ws->_uart_controller->Handle_UartAdd(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding I2C DeviceAddOrReplace messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbI2cAdds(pb_istream_t *stream, const pb_field_t *field,
                             void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_i2c_Add add_msg = ws_i2c_Add_init_zero;
  Ws->_i2c_controller->SetupAddDecodeCallbacks(&add_msg);
  if (!pb_decode(stream, ws_i2c_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "i2c");
  }
  ws_i2c_Remove remove_msg = ws_i2c_Remove_init_zero;
  remove_msg.has_descriptor = add_msg.has_descriptor;
  remove_msg.descriptor = add_msg.descriptor;
  // I2C Add replaces an existing driver itself. An explicit Remove would also
  // tear down every device behind a MUX when the MUX itself changes.
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_i2c_tag,
                          ws_i2c_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (!model->CommitAdd(Ws->_i2c_controller->Handle_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding Telemetry Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbTelemetryAdds(pb_istream_t *stream,
                                   const pb_field_t *field, void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_telemetry_Add add_msg = ws_telemetry_Add_init_zero;
  if (!pb_decode(stream, ws_telemetry_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "telemetry");
  }
  ws_telemetry_Remove remove_msg = ws_telemetry_Remove_init_zero;
  remove_msg.type = add_msg.type;
  // Telemetry Add updates an existing metric itself
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_telemetry_tag,
                          ws_telemetry_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (!model->CommitAdd(
          Ws->_telemetry_controller->Handle_TelemetryAdd(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding Display Add messages. Decodes into the
              _display_add member rather than the stack, the struct is large.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbDisplayAdds(pb_istream_t *stream, const pb_field_t *field,
                                 void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  memset(&model->_display_add, 0, sizeof(model->_display_add));
  if (!pb_decode(stream, ws_display_Add_fields, &model->_display_add)) {
    return model->SkipFailedAdd(stream, "display");
  }
  ws_display_Remove remove_msg = ws_display_Remove_init_zero;
  strncpy(remove_msg.name, model->_display_add.name,
          sizeof(remove_msg.name) - 1);
  // Display Add replaces an existing display of the same name itself
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_display_tag,
                          ws_display_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  if (!model->CommitAdd(
          Ws->_display_controller->Handle_Display_Add(&model->_display_add)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief    Callback for decoding Expander Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True, a failed add is counted and skipped.
*/
bool CheckinModel::cbExpanderAdds(pb_istream_t *stream, const pb_field_t *field,
                                  void **arg) {
  CheckinModel *model = (CheckinModel *)*arg;
  model->_adds_total++;
  pb_istream_t raw = *stream;
  ws_expander_Add add_msg = ws_expander_Add_init_zero;
  // Set up the nested i2c cfg's settings callback before decoding, same as
  // ExpanderController::Router
  Ws->_i2c_controller->SetupAddDecodeCallbacks(&add_msg.cfg_i2c);
  if (!pb_decode(stream, ws_expander_Add_fields, &add_msg)) {
    return model->SkipFailedAdd(stream, "expander");
  }
  ws_expander_Remove remove_msg = ws_expander_Remove_init_zero;
  remove_msg.has_cfg_i2c = true;
  remove_msg.cfg_i2c.has_descriptor = add_msg.cfg_i2c.has_descriptor;
  remove_msg.cfg_i2c.descriptor = add_msg.cfg_i2c.descriptor;
  CheckinAction action =
      model->ReconcileAdd(&raw, ws_signal_BrokerToDevice_expander_tag,
                          ws_expander_Remove_fields, &remove_msg);
  if (action == CHECKIN_SKIP)
    return true;
  // Expander Add rejects an address already in use, remove it first
  if (action == CHECKIN_REPLACE &&
      !Ws->_expander_controller->Handle_Remove(&remove_msg))
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to remove component to replace");
  if (!model->CommitAdd(Ws->_expander_controller->Handle_Add(&add_msg)))
    model->_adds_failed++;
  return true;
}

/*!
    @brief  Returns whether sleep is enabled in the checkin response.
    @returns True if sleep is enabled, False otherwise.
*/
bool CheckinModel::IsSleepEnabled() {
  return _CheckinB2D.payload.response.sleep_enabled;
}

/*!
    @brief  Returns the sleep configuration from the checkin response.
    @returns Pointer to sleep config if present, nullptr otherwise.
*/
ws_sleep_SleepConfig *CheckinModel::GetSleepConfig() {
  if (_CheckinB2D.payload.response.has_sleep_config) {
    return &_CheckinB2D.payload.response.sleep_config;
  }
  return nullptr;
}

/*!
    @brief  Returns the total number of GPIO pins from the checkin response.
    @returns Total GPIO pins.
*/
int32_t CheckinModel::GetTotalGPIOPins() {
  return _CheckinB2D.payload.response.total_gpio_pins;
}

/*!
    @brief  Returns the total number of analog pins from the checkin response.
    @returns Total analog pins.
*/
int32_t CheckinModel::GetTotalAnalogPins() {
  return _CheckinB2D.payload.response.total_analog_pins;
}

/*!
    @brief  Returns the reference voltage from the checkin response.
    @returns Reference voltage.
*/
float CheckinModel::GetReferenceVoltage() {
  return _CheckinB2D.payload.response.reference_voltage;
}

/*!
    @brief  Configures sleep controller based on checkin response.
*/
void CheckinModel::configureSleep() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2350)
  // Process sleep configuration if present
  if (IsSleepEnabled()) {
    ws_sleep_SleepConfig *sleep_cfg = GetSleepConfig();
    if (sleep_cfg != nullptr) {
      WS_DEBUG_PRINTLN("[app] Processing sleep configuration from checkin");
      Ws->_sleep_controller->handleSleepConfig(sleep_cfg, true);
    }
  }
#endif
}

/*!
    @brief  Returns whether a checkin response has been received.
    @returns True if response received, False otherwise.
*/
bool CheckinModel::GotResponse() { return _got_response; }

/*!
    @brief  Checks that a checkin response hash buffer holds a hash.
    @param  sha
            CHECKIN_SHA_LEN byte hash buffer.
    @returns True if the buffer is not null and not all zero, False if it is
             missing, was never computed or hashing is unsupported.
*/
static bool IsShaValid(const uint8_t *sha) {
  if (sha == nullptr)
    return false;
  for (size_t i = 0; i < CHECKIN_SHA_LEN; i++) {
    if (sha[i] != 0)
      return true;
  }
  return false;
}

/*!
    @brief  Hashes the raw, not yet decoded, checkin message and compares it
            against the last fully handled checkin response.
    @param  stream
            Incoming data stream positioned at the start of the checkin
            message. Must be a buffer stream (pb_istream_from_buffer), as
            used for both MQTT and SD card messages. Not consumed.
    @returns True if the response differs from the last one (or there is no
             previous response, or hashing is unsupported), False if it is
             identical.
*/
bool CheckinModel::ResponseChanged(pb_istream_t *stream) {
  // A buffer stream's state points at its next unread byte
  if (!sha256((const uint8_t *)stream->state, stream->bytes_left,
              _sha_checkin_response))
    memset(_sha_checkin_response, 0, CHECKIN_SHA_LEN);
  if (!IsShaValid(_sha_checkin_response) ||
      !IsShaValid(last_sha_checkin_response))
    return true;
  return memcmp(_sha_checkin_response, last_sha_checkin_response,
                CHECKIN_SHA_LEN) != 0;
}

/*!
    @brief  Saves the hash of this checkin response as the last fully handled
            response. Call only once the response has been applied.
*/
void CheckinModel::SaveResponseHash() {
  if (!IsShaValid(_sha_checkin_response))
    return;
  memset(last_sha_checkin_response, 0, CHECKIN_SHA_LEN);
  memcpy(last_sha_checkin_response, _sha_checkin_response, CHECKIN_SHA_LEN);
}

/*!
    @brief  Decides how to apply a component Add from a checkin response by
            comparing it against the components applied from earlier
            responses. Follow with CommitAdd() unless CHECKIN_SKIP.
    @param  raw
            Copy of the Add's buffer stream, taken before it was decoded.
    @param  type
            BrokerToDevice payload tag of the component.
    @param  remove_fields
            nanopb fields of the component's Remove message.
    @param  remove_msg
            Remove message identifying the component, built from the Add.
    @returns CHECKIN_SKIP if the component is unchanged (or in the
             RemoveStaleComponents() mark pass), CHECKIN_REPLACE if it exists
             with a different Add, CHECKIN_ADD otherwise.
*/
CheckinAction CheckinModel::ReconcileAdd(const pb_istream_t *raw,
                                         pb_size_t type,
                                         const pb_msgdesc_t *remove_fields,
                                         const void *remove_msg) {
  _has_pending = false;
  _pending.type = type;
  _pending.seen = false;
  size_t sz = 0;
  bool keyed =
      sha256((const uint8_t *)raw->state, raw->bytes_left, _pending.add_sha) &&
      pb_get_encoded_size(&sz, remove_fields, remove_msg);
  if (keyed) {
    _pending.remove.resize(sz);
    pb_ostream_t stream = pb_ostream_from_buffer(_pending.remove.data(), sz);
    keyed = pb_encode(&stream, remove_fields, remove_msg);
  }
  _pending_idx = keyed ? findComponent(type, _pending.remove) : -1;

  // Mark pass: only record which components are still present
  if (_mark_only) {
    if (_pending_idx >= 0)
      checkin_components[_pending_idx].seen = true;
    return CHECKIN_SKIP;
  }
  if (!keyed)
    return CHECKIN_ADD;
  _has_pending = true;

  if (_pending_idx < 0)
    return CHECKIN_ADD;
  CheckinComponent &existing = checkin_components[_pending_idx];
  if (memcmp(existing.add_sha, _pending.add_sha, CHECKIN_SHA_LEN) == 0) {
    _has_pending = false;
    WS_DEBUG_PRINTLN("[checkin] Component unchanged, skipping");
    return CHECKIN_SKIP;
  }
  WS_DEBUG_PRINTLN("[checkin] Component changed, replacing");
  return CHECKIN_REPLACE;
}

/*!
    @brief  Records the outcome of applying the component Add passed to the
            last ReconcileAdd(), so later checkin responses reconcile
            against it.
    @param  added
            Result of the controller's Add handler.
    @returns added, so callbacks can return it directly.
*/
bool CheckinModel::CommitAdd(bool added) {
  if (!_has_pending)
    return added;
  _has_pending = false;
  if (_pending_idx < 0) {
    if (added)
      checkin_components.push_back(_pending);
  } else if (added) {
    memcpy(checkin_components[_pending_idx].add_sha, _pending.add_sha,
           CHECKIN_SHA_LEN);
  } else {
    // The old component was removed (or its removal failed and was logged)
    // and its replacement failed, so stop tracking it
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to replace component");
    checkin_components.erase(checkin_components.begin() + _pending_idx);
  }
  return added;
}

/*!
    @brief  Forgets the components applied from earlier checkin responses,
            so this response's adds are all applied and recorded afresh.
*/
void CheckinModel::ForgetComponents() { checkin_components.clear(); }

/*!
    @brief  Marks this checkin as one made after a light sleep wake, the only
            case where a response is reconciled against earlier ones.
    @param  wake
            True if checking in after a light sleep wake.
*/
void CheckinModel::SetWakeCheckin(bool wake) { _wake_checkin = wake; }

/*!
    @brief  Returns whether this checkin was made after a light sleep wake.
    @returns True if checking in after a light sleep wake, False otherwise.
*/
bool CheckinModel::IsWakeCheckin() { return _wake_checkin; }

/*!
    @brief  Removes components applied from earlier checkin responses that
            are not in this response. Call before ProcessComponentAdds(), so
            a stale component never tears down a pin a new one has taken.
    @param  stream
            Incoming data stream positioned at the start of the checkin
            message. Not consumed.
*/
void CheckinModel::RemoveStaleComponents(pb_istream_t *stream) {
  if (checkin_components.empty())
    return;

  // Mark pass: decode a copy of the response without applying any component
  for (size_t i = 0; i < checkin_components.size(); i++)
    checkin_components[i].seen = false;
  pb_istream_t mark_stream = *stream;
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));
  _CheckinB2D.cb_payload.funcs.decode = &cbSetupResponse;
  _CheckinB2D.cb_payload.arg = this;
  _mark_only = true;
  bool decoded = pb_decode(&mark_stream, ws_checkin_B2D_fields, &_CheckinB2D);
  _mark_only = false;
  // ProcessResponse() reports invalid responses, never remove on one
  if (!decoded || _CheckinB2D.which_payload != ws_checkin_B2D_response_tag ||
      _CheckinB2D.payload.response.response !=
          ws_checkin_Response_Response_R_OK)
    return;

  for (size_t i = 0; i < checkin_components.size();) {
    CheckinComponent &c = checkin_components[i];
    if (c.seen) {
      i++;
      continue;
    }
    WS_DEBUG_PRINTLN("[checkin] Removing component no longer in checkin");
    switch (c.type) {
    case ws_signal_BrokerToDevice_digital_tag: {
      ws_digital_Remove msg = ws_digital_Remove_init_zero;
      if (decodeRemove(c, ws_digital_Remove_fields, &msg))
        (void)Ws->digital_controller->Handle_Digital_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_analog_tag: {
      ws_analog_Remove msg = ws_analog_Remove_init_zero;
      if (decodeRemove(c, ws_analog_Remove_fields, &msg))
        (void)Ws->analog_controller->Handle_Analog_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_servo_tag: {
      ws_servo_Remove msg = ws_servo_Remove_init_zero;
      if (decodeRemove(c, ws_servo_Remove_fields, &msg))
        (void)Ws->_servo_controller->Handle_Servo_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_pwm_tag: {
      ws_pwm_Remove msg = ws_pwm_Remove_init_zero;
      if (decodeRemove(c, ws_pwm_Remove_fields, &msg))
        (void)Ws->_pwm_controller->Handle_PWM_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_pixels_tag: {
      ws_pixels_Remove msg = ws_pixels_Remove_init_zero;
      if (decodeRemove(c, ws_pixels_Remove_fields, &msg))
        (void)Ws->_pixels_controller->Handle_Pixels_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_ds18x20_tag: {
      ws_ds18x20_Remove msg = ws_ds18x20_Remove_init_zero;
      if (decodeRemove(c, ws_ds18x20_Remove_fields, &msg))
        (void)Ws->_ds18x20_controller->Handle_Ds18x20Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_uart_tag: {
      ws_uart_Remove msg = ws_uart_Remove_init_zero;
      if (decodeRemove(c, ws_uart_Remove_fields, &msg))
        (void)Ws->_uart_controller->Handle_UartRemove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_i2c_tag: {
      ws_i2c_Remove msg = ws_i2c_Remove_init_zero;
      if (decodeRemove(c, ws_i2c_Remove_fields, &msg))
        (void)Ws->_i2c_controller->Handle_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_telemetry_tag: {
      ws_telemetry_Remove msg = ws_telemetry_Remove_init_zero;
      if (decodeRemove(c, ws_telemetry_Remove_fields, &msg))
        (void)Ws->_telemetry_controller->Handle_TelemetryRemove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_display_tag: {
      ws_display_Remove msg = ws_display_Remove_init_zero;
      if (decodeRemove(c, ws_display_Remove_fields, &msg))
        (void)Ws->_display_controller->Handle_Display_Remove(&msg);
      break;
    }
    case ws_signal_BrokerToDevice_expander_tag: {
      ws_expander_Remove msg = ws_expander_Remove_init_zero;
      if (decodeRemove(c, ws_expander_Remove_fields, &msg))
        (void)Ws->_expander_controller->Handle_Remove(&msg);
      break;
    }
    default:
      break;
    }
    checkin_components.erase(checkin_components.begin() + i);
  }
}

/*!
    @brief  Copies a received checkin response so it can be applied after
            the MQTT callback returns. Applying it publishes (device and
            component errors), and publishing inside the callback can
            consume another packet's reply.
    @param  stream
            Incoming data stream positioned at the start of the checkin
            message. Consumed.
    @returns True if stored, False if the stream could not be read.
*/
bool CheckinModel::StoreResponse(pb_istream_t *stream) {
  _stored_response.resize(stream->bytes_left);
  if (!pb_read(stream, _stored_response.data(), _stored_response.size())) {
    _stored_response.clear();
    return false;
  }
  _got_response = true;
  return true;
}

/*!
    @brief  Returns a stream over the checkin response stored by
            StoreResponse().
    @returns Buffer stream over the stored response.
*/
pb_istream_t CheckinModel::GetStoredResponse() {
  return pb_istream_from_buffer(_stored_response.data(),
                                _stored_response.size());
}

/*!
    @brief  Publishes a ws_checkin_Complete message to the broker,
            signaling that the device has completed the checkin routine.
    @returns True if the message was published successfully, False otherwise.
*/
bool CheckinModel::Complete() {
  // Create a fresh D2B wrapper message
  ws_checkin_D2B completeMsg = ws_checkin_D2B_init_default;

  // Create and fill the ws_checkin_Complete message
  completeMsg.which_payload = ws_checkin_D2B_complete_tag;
  completeMsg.payload.complete.dummy_field = '\0';

  // Encode the D2B wrapper message
  size_t CheckinD2BSz;
  if (!pb_get_encoded_size(&CheckinD2BSz, ws_checkin_D2B_fields,
                           &completeMsg)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to get encoded size of CheckinD2B message!");
    return false;
  }

  uint8_t buf[CheckinD2BSz];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&msg_stream, ws_checkin_D2B_fields, &completeMsg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Unable to encode CheckinD2B message!");
    return false;
  }

  // Publish the message
  if (!Ws->PublishD2b(ws_signal_DeviceToBroker_checkin_tag, &completeMsg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Unable to publish Complete message!");
    return false;
  }
  return true;
}