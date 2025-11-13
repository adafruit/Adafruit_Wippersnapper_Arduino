/*!
 * @file src/components/checkin/model.cpp
 *
 * Model for the Wippersnapper checkin proto API.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "model.h"

/*!
    @brief  CheckinModel constructor
*/
CheckinModel::CheckinModel() {
  memset(&_CheckinRequest, 0, sizeof(_CheckinRequest));
  memset(&_CheckinResponse, 0, sizeof(_CheckinResponse));
  // no-op
}

/*!
    @brief  CheckinModel destructor
*/
CheckinModel::~CheckinModel() {
  memset(&_CheckinRequest, 0, sizeof(_CheckinRequest));
  memset(&_CheckinResponse, 0, sizeof(_CheckinResponse));
}

bool CheckinModel::EncodeD2bCheckinRequest(const char *hw_uid,
                                           const char *fw_ver) {
  // Validate input len
  if (strlen(hw_uid) >=
          sizeof(_CheckinD2B.payload.checkin_request.hardware_uid) ||
      strlen(fw_ver) >=
          sizeof(_CheckinD2B.payload.checkin_request.firmware_version)) {
    return false;
  }

  // Zero-out the message envelope
  memset(&_CheckinD2B, 0, sizeof(_CheckinD2B));
  // Set which_payload
  _CheckinD2B.which_payload =
      wippersnapper_checkin_CheckinD2B_checkin_request_tag;
  // Safely fill the CheckinRequest sub-message payload
  strncpy(_CheckinD2B.payload.checkin_request.hardware_uid, hw_uid,
          sizeof(_CheckinD2B.payload.checkin_request.hardware_uid) - 1);
  _CheckinD2B.payload.checkin_request
      .hardware_uid[sizeof(_CheckinD2B.payload.checkin_request.hardware_uid) -
                    1] = '\0';
  strncpy(_CheckinD2B.payload.checkin_request.firmware_version, fw_ver,
          sizeof(_CheckinD2B.payload.checkin_request.firmware_version) - 1);
  _CheckinD2B.payload.checkin_request.firmware_version
      [sizeof(_CheckinD2B.payload.checkin_request.firmware_version) - 1] = '\0';

  // Obtain size of the CheckinD2B message
  size_t CheckinD2BSz;
  if (!pb_get_encoded_size(
          &CheckinD2BSz, wippersnapper_checkin_CheckinD2B_fields, &_CheckinD2B))
    return false;
  // Create a temporary buffer for holding the CheckinD2B message
  uint8_t buf[CheckinD2BSz];
  // Create a stream that will write to buf
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  // Attempt to encode the message
  return pb_encode(&msg_stream, wippersnapper_checkin_CheckinD2B_fields,
                   &_CheckinD2B);
}


bool CheckinModel::DecodeB2d(pb_istream_t *stream) {
    // Zero-out the CheckinB2D message
    memset(&_CheckinB2D, 0, sizeof(_CheckinB2D));
    // Decode the message
    if (!pb_decode(stream, wippersnapper_checkin_CheckinB2D_fields, &_CheckinB2D))
      return false;

    // Attempt to get the payload and dispatch to the appropriate handler
    if (_CheckinB2D.which_payload == wippersnapper_checkin_CheckinB2D_checkin_response_tag) {
      // TODO: Refactor this code outwards to a handler func.
      // Parse the CheckinResponse sub-message
      _total_analog_pins = _CheckinB2D.payload.checkin_response.total_analog_pins;
      _total_gpio_pins = _CheckinB2D.payload.checkin_response.total_gpio_pins;
      _reference_voltage = _CheckinB2D.payload.checkin_response.reference_voltage;
      // Okay, now let's look at component_adds[32] array
      for (pb_size_t i = 0; i < _CheckinB2D.payload.checkin_response.component_adds_count; i++) {
        // For now, just print out what component_adds we have
        wippersnapper_checkin_ComponentAdd *compAdd = &_CheckinB2D.payload.checkin_response.component_adds[i];
        if (compAdd->which_payload == wippersnapper_checkin_ComponentAdd_digitalio_tag) {
          // Process a DigitalIOAdd component add message
          WS_DEBUG_PRINTLN("INFO: Found DigitalIOAdd component_add in CheckinResponse.");
          WsV2.digital_io_controller->Handle_DigitalIO_Add(&compAdd->payload.digitalio);
        } else {
          // Component type not found
          WS_DEBUG_PRINTLN("WARNING: Unknown component_add type in CheckinResponse!");
        }
      }
    }

    return true;
}

/*!
    @brief  Fills and creates a CheckinRequest message
    @param  hardware_uid
            Hardware's unique identifier.
    @param  firmware_version
            WipperSnapper firmware version.
*/
void CheckinModel::CreateCheckinRequest(const char *hardware_uid,
                                        const char *firmware_version) {
  strcpy(_CheckinRequest.hardware_uid, hardware_uid);
  strcpy(_CheckinRequest.firmware_version, firmware_version);
}

/*!
    @brief  Encodes a CheckinRequest message
    @returns True if the message was successfully encoded,
             False otherwise.
*/
bool CheckinModel::EncodeCheckinRequest() {
  // Obtain size of the CheckinRequest message
  size_t CheckinRequestSz;
  if (!pb_get_encoded_size(&CheckinRequestSz,
                           wippersnapper_checkin_CheckinRequest_fields,
                           &_CheckinRequest))
    return false;
  // Create a buffer for holding the CheckinRequest message
  uint8_t buf[CheckinRequestSz];

  // Create a stream that will write to buf
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  // Encode the message
  return pb_encode(&msg_stream, wippersnapper_checkin_CheckinRequest_fields,
                   &_CheckinRequest);
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
    @brief  Gets the CheckinD2B message
    @returns CheckinD2B message pointer.
*/
wippersnapper_checkin_CheckinD2B *CheckinModel::getD2bCheckinRequest() {
  return &_CheckinD2B;
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
