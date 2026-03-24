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

// TODO: Remove all debug prints after we are confident during testing

/*!
    @brief  CheckinModel constructor
*/
CheckinModel::CheckinModel() { _got_response = false; }

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
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_checkin_tag, &_CheckinD2B)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to publish CheckinRequest message!");
    return false;
  }
  WS_DEBUG_PRINTLN("...Published!");
  return true;
}

/*!
    @brief  Decodes a ws_checkin_Response message from the broker.
    @param  stream
            Incoming data stream from buffer.
    @returns True if decoded successfully, False otherwise.
*/
bool CheckinModel::ProcessResponse(pb_istream_t *stream) {
  // Zero-out the B2D wrapper message
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));

  // Set up the pre-decode callback (cb_payload) which will be called BEFORE
  // the Response submessage is decoded. This allows us to set up the
  // component_adds callback before nanopb initializes the Response struct.
  _CheckinB2D.cb_payload.funcs.decode = &cbSetupResponse;
  _CheckinB2D.cb_payload.arg = this;

  // Decode the B2D wrapper message
  if (!pb_decode(stream, ws_checkin_B2D_fields, &_CheckinB2D)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to decode CheckinB2D message from broker!");
    return false;
  }

  // Validate the response message type
  if (_CheckinB2D.which_payload != ws_checkin_B2D_response_tag) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: CheckinB2D message from broker is not a Response!");
    return false;
  }

  // Validate the response content
  if (_CheckinB2D.payload.response.response !=
      ws_checkin_Response_Response_R_OK) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Invalid checkin_Response!");
    return false;
  }

  // Set flag to indicate response received
  _got_response = true;
  return true;
}

/*!
    @brief  Configures controllers limits based on board definition
*/
void CheckinModel::ConfigureControllers() {
  Ws.digital_io_controller->SetMaxDigitalPins(
      _CheckinB2D.payload.response.total_gpio_pins);
  Ws.analogio_controller->SetRefVoltage(
      _CheckinB2D.payload.response.reference_voltage);
  Ws.analogio_controller->SetTotalAnalogPins(
      _CheckinB2D.payload.response.total_analog_pins);
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
  response->component_adds.digitalio_adds.funcs.decode = &cbDigitalIOAdds;
  response->component_adds.digitalio_adds.arg = model;

  response->component_adds.analogio_adds.funcs.decode = &cbAnalogIOAdds;
  response->component_adds.analogio_adds.arg = model;

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

  // Return true WITHOUT consuming the stream - nanopb will continue
  // to decode the Response submessage with our callback now set up
  return true;
}

/*!
    @brief    Callback for decoding DigitalIO Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbDigitalIOAdds(pb_istream_t *stream,
                                   const pb_field_t *field, void **arg) {
  ws_digitalio_Add add_msg = ws_digitalio_Add_init_zero;
  if (!pb_decode(stream, ws_digitalio_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode digitalio add");
    return false;
  }
  return Ws.digital_io_controller->Handle_DigitalIO_Add(&add_msg);
}

/*!
    @brief    Callback for decoding AnalogIO Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbAnalogIOAdds(pb_istream_t *stream, const pb_field_t *field,
                                  void **arg) {
  ws_analogio_Add add_msg = ws_analogio_Add_init_zero;
  if (!pb_decode(stream, ws_analogio_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode analogio add");
    return false;
  }
  return Ws.analogio_controller->Handle_AnalogIOAdd(&add_msg);
}

/*!
    @brief    Callback for decoding Servo Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbServoAdds(pb_istream_t *stream, const pb_field_t *field,
                               void **arg) {
  ws_servo_Add add_msg = ws_servo_Add_init_zero;
  if (!pb_decode(stream, ws_servo_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode servo add");
    return false;
  }
  return Ws._servo_controller->Handle_Servo_Add(&add_msg);
}

/*!
    @brief    Callback for decoding PWM Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbPWMAdds(pb_istream_t *stream, const pb_field_t *field,
                             void **arg) {
  ws_pwm_Add add_msg = ws_pwm_Add_init_zero;
  if (!pb_decode(stream, ws_pwm_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode pwm add");
    return false;
  }
  return Ws._pwm_controller->Handle_PWM_Add(&add_msg);
}

/*!
    @brief    Callback for decoding Pixels Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbPixelsAdds(pb_istream_t *stream, const pb_field_t *field,
                                void **arg) {
  ws_pixels_Add add_msg = ws_pixels_Add_init_zero;
  if (!pb_decode(stream, ws_pixels_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode pixels add");
    return false;
  }
  return Ws._pixels_controller->Handle_Pixels_Add(&add_msg);
}

/*!
    @brief    Callback for decoding DS18x20 Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbDs18x20Adds(pb_istream_t *stream, const pb_field_t *field,
                                 void **arg) {
  ws_ds18x20_Add add_msg = ws_ds18x20_Add_init_zero;
  if (!pb_decode(stream, ws_ds18x20_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode ds18x20 add");
    return false;
  }
  return Ws._ds18x20_controller->Handle_Ds18x20Add(&add_msg);
}

/*!
    @brief    Callback for decoding UART Add messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbUartAdds(pb_istream_t *stream, const pb_field_t *field,
                              void **arg) {
  ws_uart_Add add_msg = ws_uart_Add_init_zero;
  if (!pb_decode(stream, ws_uart_Add_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode uart add");
    return false;
  }
  return Ws._uart_controller->Handle_UartAdd(&add_msg);
}

/*!
    @brief    Callback for decoding I2C DeviceAddOrReplace messages.
    @param    stream  Incoming data stream from buffer.
    @param    field   Protobuf message's tag type.
    @param    arg     Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbI2cAdds(pb_istream_t *stream, const pb_field_t *field,
                             void **arg) {
  ws_i2c_DeviceAddOrReplace add_msg = ws_i2c_DeviceAddOrReplace_init_zero;
  if (!pb_decode(stream, ws_i2c_DeviceAddOrReplace_fields, &add_msg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to decode i2c add");
    return false;
  }
  return Ws._i2c_controller->Handle_I2cDeviceAddOrReplace(&add_msg);
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
    @brief  Configures sleep controller based on checkin response.
*/
void CheckinModel::configureSleep() {
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2350)
  // Process sleep configuration if present
  if (IsSleepEnabled()) {
    ws_sleep_SleepConfig *sleep_cfg = GetSleepConfig();
    if (sleep_cfg != nullptr) {
      WS_DEBUG_PRINTLN("[app] Processing sleep configuration from checkin");
      Ws._sleep_controller->handleSleepConfig(sleep_cfg, true);
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
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_checkin_tag, &completeMsg)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Unable to publish Complete message!");
    return false;
  }
  return true;
}