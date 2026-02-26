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
    @brief    Pre-decode callback for setting up component_adds callback.
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

  // Set up the component_adds callback before nanopb decodes the Response
  CheckinModel *model = (CheckinModel *)*arg;
  response->component_adds.funcs.decode = &cbComponentAdds;
  response->component_adds.arg = model;

  // Return true WITHOUT consuming the stream - nanopb will continue
  // to decode the Response submessage with our callback now set up
  return true;
}

/*!
    @brief    Callback for decoding ComponentAdd messages during Checkin
              Response processing.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
bool CheckinModel::cbComponentAdds(pb_istream_t *stream,
                                   const pb_field_t *field, void **arg) {
  // Attempt to alloc. on the heap to avoid stack overflow (ComponentAdd is a
  // ~2.4K struct)
  ws_checkin_ComponentAdd *component = new ws_checkin_ComponentAdd();
  if (component == nullptr) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Failed to allocate ComponentAdd!");
    return false;
  }

  if (!pb_decode(stream, ws_checkin_ComponentAdd_fields, component)) {
    WS_DEBUG_PRINTLN(
        "[checkin] SYNC ERROR: Unable to decode ComponentAdd message!");
    delete component;
    return false;
  }

  bool result = true;
  switch (component->which_payload) {
  case ws_checkin_ComponentAdd_digitalio_tag:
    result = Ws.digital_io_controller->Handle_DigitalIO_Add(
        &component->payload.digitalio);
    break;
  case ws_checkin_ComponentAdd_analogio_tag:
    result = Ws.analogio_controller->Handle_AnalogIOAdd(
        &component->payload.analogio);
    break;
  case ws_checkin_ComponentAdd_servo_tag:
    result = Ws._servo_controller->Handle_Servo_Add(&component->payload.servo);
    break;
  case ws_checkin_ComponentAdd_pwm_tag:
    result = Ws._pwm_controller->Handle_PWM_Add(&component->payload.pwm);
    break;
  case ws_checkin_ComponentAdd_pixels_tag:
    result =
        Ws._pixels_controller->Handle_Pixels_Add(&component->payload.pixels);
    break;
  case ws_checkin_ComponentAdd_ds18x20_tag:
    result =
        Ws._ds18x20_controller->Handle_Ds18x20Add(&component->payload.ds18x20);
    break;
  case ws_checkin_ComponentAdd_uart_tag:
    result = Ws._uart_controller->Handle_UartAdd(&component->payload.uart);
    break;
  case ws_checkin_ComponentAdd_i2c_tag:
    result = Ws._i2c_controller->Handle_I2cDeviceAddOrReplace(
        &component->payload.i2c);
    break;
  default:
    WS_DEBUG_PRINTLN("UNKNOWN COMPONENT TYPE!");
    result = false;
    break;
  }

  // Clean up heap allocation for this message
  delete component;
  return result;
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