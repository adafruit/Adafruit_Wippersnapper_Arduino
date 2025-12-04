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
  if (!WsV2.PublishD2b(ws_signal_DeviceToBroker_checkin_tag, &_CheckinD2B)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to publish CheckinRequest message!");
    return false;
  }
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

  // Configure the callback for the component_adds field
  _CheckinB2D.payload.response.component_adds.funcs.decode = &cbComponentAdds;
  _CheckinB2D.payload.response.component_adds.arg = this;

  // Decode the B2d wrapper message
  // NOTE: This also decodes the component_adds message
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

  _got_response = true; // Set flag to indicate response received
  return true;
}

/*!
    @brief  Configures controllers limits based on board definition
*/
void CheckinModel::ConfigureControllers() {
  WsV2.digital_io_controller->SetMaxDigitalPins(
      _CheckinB2D.payload.response.total_gpio_pins);
  WsV2.analogio_controller->SetRefVoltage(
      _CheckinB2D.payload.response.reference_voltage);
  WsV2.analogio_controller->SetTotalAnalogPins(
      _CheckinB2D.payload.response.total_analog_pins);
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
  ws_checkin_ComponentAdd component = ws_checkin_ComponentAdd_init_default;
  if (!pb_decode(stream, ws_checkin_ComponentAdd_fields, &component)) {
    WS_DEBUG_PRINTLN("[checkin] ERROR: Unable to decode ComponentAdd message!");
  }

  // TODO: Remove debug prints after testing
  WS_DEBUG_PRINT("[checkin] Adding component: ");
  switch (component.which_payload) {
  case ws_checkin_ComponentAdd_digitalio_tag:
    WS_DEBUG_PRINTLN("DigitalIO");
    return WsV2.digital_io_controller->Handle_DigitalIO_Add(
        &component.payload.digitalio);
  case ws_checkin_ComponentAdd_analogio_tag:
    WS_DEBUG_PRINTLN("AnalogIO");
    return WsV2.analogio_controller->Handle_AnalogIOAdd(
        &component.payload.analogio);
  case ws_checkin_ComponentAdd_servo_tag:
    WS_DEBUG_PRINTLN("Servo");
    return WsV2._servo_controller->Handle_Servo_Add(&component.payload.servo);
  case ws_checkin_ComponentAdd_pwm_tag:
    WS_DEBUG_PRINTLN("PWM");
    return WsV2._pwm_controller->Handle_PWM_Add(&component.payload.pwm);
  case ws_checkin_ComponentAdd_pixels_tag:
    WS_DEBUG_PRINTLN("Pixels");
    return WsV2._pixels_controller->Handle_Pixels_Add(
        &component.payload.pixels);
  case ws_checkin_ComponentAdd_ds18x20_tag:
    WS_DEBUG_PRINTLN("DS18x20");
    return WsV2._ds18x20_controller->Handle_Ds18x20Add(
        &component.payload.ds18x20);
  case ws_checkin_ComponentAdd_uart_tag:
    WS_DEBUG_PRINTLN("UART");
    return WsV2._uart_controller->Handle_UartAdd(&component.payload.uart);
  case ws_checkin_ComponentAdd_i2c_tag:
    WS_DEBUG_PRINTLN("I2C");
    return WsV2._i2c_controller->Handle_I2cDeviceAddOrReplace(
        &component.payload.i2c);
  default:
    WS_DEBUG_PRINTLN("UNKNOWN COMPONENT TYPE!");
    break;
  }

  return true;
}

/*!
    @brief  Returns whether a checkin response has been received.
    @returns True if response received, False otherwise.
*/
bool CheckinModel::GotResponse() { return _got_response; }