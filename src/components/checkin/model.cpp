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
CheckinModel::CheckinModel() {
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));
  memset(&_CheckinD2B, 0, sizeof(_CheckinD2B));
  _got_response = false;
}

/*!
    @brief  CheckinModel destructor
*/
CheckinModel::~CheckinModel() {
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));
  memset(&_CheckinD2B, 0, sizeof(_CheckinD2B));
  _got_response = false;
}

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

  // Decode the B2d wrapper message
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
    @brief  Adds components from the broker
    @returns True if components were added successfully, False otherwise.
*/
bool CheckinModel::AddComponents() {
  // early-out if no components to add
  if (_CheckinB2D.payload.response.component_adds_count == 0) {
    WS_DEBUG_PRINTLN("[checkin] No components to add!");
    return true;
  }

  WS_DEBUG_PRINT("[checkin] Adding ");
  WS_DEBUG_PRINT(_CheckinB2D.payload.response.component_adds_count);
  WS_DEBUG_PRINTLN(" components...");

  // Iterate through each component add message
  for (pb_size_t i = 0; i < _CheckinB2D.payload.response.component_adds_count;
       i++) {
    const ws_checkin_ComponentAdd *comp_add =
        &_CheckinB2D.payload.response.component_adds[i];
    WS_DEBUG_PRINTLN("[checkin] Adding component from broker...");
    switch (comp_add->which_payload) {
    case ws_checkin_ComponentAdd_digitalio_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding DigitalIO component...");
      break;
    case ws_checkin_ComponentAdd_analogio_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding AnalogIO component...");
      break;
    case ws_checkin_ComponentAdd_servo_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding Servo component...");
      break;
    case ws_checkin_ComponentAdd_pwm_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding PWM component...");
      break;
    case ws_checkin_ComponentAdd_pixels_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding Pixels component...");
      break;
    case ws_checkin_ComponentAdd_ds18x20_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding DS18X20 component...");
      break;
    case ws_checkin_ComponentAdd_uart_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding UART component...");
      break;
    case ws_checkin_ComponentAdd_i2c_tag:
      // TODO
      WS_DEBUG_PRINTLN("[checkin] Adding I2C component...");
      break;
    default:
      WS_DEBUG_PRINTLN(
          "[checkin] WARNING: Unknown component add type from broker!");
      break;
    }
  }

  return true;
}

/*!
    @brief  Returns whether a checkin response has been received.
    @returns True if response received, False otherwise.
*/
bool CheckinModel::GotResponse() { return _got_response; }