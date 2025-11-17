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

/*!
    @brief  CheckinModel constructor
*/
CheckinModel::CheckinModel() {
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));
  memset(&_CheckinD2B, 0, sizeof(_CheckinD2B));
}

/*!
    @brief  CheckinModel destructor
*/
CheckinModel::~CheckinModel() {
  memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));
  memset(&_CheckinD2B, 0, sizeof(_CheckinD2B));
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
  if (!WsV2.PublishSignal(ws_signal_DeviceToBroker_checkin_tag, &_CheckinD2B)) {
    WS_DEBUG_PRINTLN(
        "[checkin] ERROR: Unable to publish CheckinRequest message!");
    return false;
  }
  return true;
}

/*!
    @brief  Decodes a CheckinResponse message
    @param  stream
            Incoming data stream from buffer.
    @returns True if the message was successfully decoded,
             False otherwise.
*/
bool CheckinModel::DecodeCheckinResponse(pb_istream_t *stream) {
  memset(&_CheckinResponse, 0, sizeof(_CheckinResponse));
  return pb_decode(stream, wippersnapper_checkin_CheckinResponse_fields,
                   &_CheckinResponse);
}

/*!
    @brief  Parses the fields of a CheckinResponse message.
*/
void CheckinModel::ParseCheckinResponse() {
  setCheckinResponse(_CheckinResponse.response);
  setTotalGPIOPins(_CheckinResponse.total_gpio_pins);
  setTotalAnalogPins(_CheckinResponse.total_analog_pins);
  setReferenceVoltage(_CheckinResponse.reference_voltage);
}

/*!
    @brief  Sets the CheckinResponse message's Response field
    @param  response
            CheckinResponse message.
*/
void CheckinModel::setCheckinResponse(
    wippersnapper_checkin_CheckinResponse_Response response) {
  _response = _CheckinResponse.response;
}

/*!
    @brief  Gets the CheckinResponse message's Response field
    @returns CheckinResponse message.
*/
wippersnapper_checkin_CheckinResponse_Response
CheckinModel::getCheckinResponse() {
  return _response;
};

/*!
    @brief  Gets the CheckinRequest message
    @returns CheckinRequest message.
*/
wippersnapper_checkin_CheckinRequest *CheckinModel::getCheckinRequest() {
  return &_CheckinRequest;
}

/*!
    @brief  Sets the CheckinResponse message's total GPIO pins field
    @param  total_gpio_pins
            Total number of GPIO pins.
*/
void CheckinModel::setTotalGPIOPins(int32_t total_gpio_pins) {
  _total_gpio_pins = total_gpio_pins;
}

/*!
    @brief  Gets the CheckinResponse message's total GPIO pins field
    @returns Total number of GPIO pins.
*/
int32_t CheckinModel::getTotalGPIOPins() { return _total_gpio_pins; }

/*!
    @brief  Sets the CheckinResponse message's total analog pins field
    @param  total_analog_pins
            Total number of analog pins.
*/
void CheckinModel::setTotalAnalogPins(int32_t total_analog_pins) {
  _total_analog_pins = total_analog_pins;
}

/*!
    @brief  Gets the CheckinResponse message's total analog pins field
    @returns Total number of analog pins.
*/
int32_t CheckinModel::getTotalAnalogPins() { return _total_analog_pins; }

/*!
    @brief  Sets the CheckinResponse message's reference voltage field
    @param  reference_voltage
            Reference voltage.
*/
void CheckinModel::setReferenceVoltage(float reference_voltage) {
  _reference_voltage = reference_voltage;
}

/*!
    @brief  Gets the CheckinResponse message's reference voltage field
    @returns Reference voltage.
*/
float CheckinModel::getReferenceVoltage() { return _reference_voltage; }