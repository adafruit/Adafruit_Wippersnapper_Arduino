/*!
 * @file Wippersnapper_Register.cpp
 *
 * API for registering hardware with the Adafruit.io
 * WipperSnapper broker.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "Wippersnapper.h"

extern Wippersnapper WS;

bool Wippersnapper::encodeRegistrationReq() {
  WS_DEBUG_PRINT("Encoding registration msg...");
  _status = true;

  // Create message object
  wippersnapper_description_v1_CreateDescriptionRequest _message =
      wippersnapper_description_v1_CreateDescriptionRequest_init_zero;

  // Set UID
  _machine_name = WS._boardId;
  _uid = atoi(WS.sUID);

  // Set machine_name
  strcpy(_message.machine_name, _machine_name);
  _message.mac_addr = _uid;

  // Set version
  strcpy(_message.str_version, WS_VERSION);

  // encode message
  pb_ostream_t _msg_stream =
      pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

  _status = pb_encode(
      &_msg_stream,
      wippersnapper_description_v1_CreateDescriptionRequest_fields, &_message);
  _message_len = _msg_stream.bytes_written;

  // verify message
  if (!_status)
    return _status
  return _status;
}

/************************************************************/
/*!
    @brief    Publishes a registration message to the
                Wippersnapper broker.
*/
/************************************************************/
void Wippersnapper::publishRegistrationReq() {
  WS.publish(WS._topic_description, _message_buffer, _message_len, 1);
  WS._boardStatus = WS_BOARD_DEF_SENT;
}

/************************************************************/
/*!
    @brief    Performs a blocking polling loop to obtain the
              registration message response from the broker.
              If not obtained, the WDT will time out and
              reset the hardware.
*/
/************************************************************/
void Wippersnapper::pollRegistrationResp() {
  // set the WDT
  WS.feedWDT();
  // poll and check until we WDT reset
  while (WS._boardStatus != WS_BOARD_DEF_OK) {
    WS._mqtt->processPackets(500);
    WS_DEBUG_PRINTLN("ERROR: Did not get registration message from broker, retrying...");
    WS.setStatusLEDColor(LED_ERROR);
  }
}

void Wippersnapper::decodeRegistrationResp(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("-> Registration Message");
  uint8_t buffer[len];
  memcpy(buffer, data, len);

  // init. CreateDescriptionResponse message
  wippersnapper_description_v1_CreateDescriptionResponse message =
      wippersnapper_description_v1_CreateDescriptionResponse_init_zero;

  // create input stream for buffer
  pb_istream_t stream = pb_istream_from_buffer(buffer, len);
  // decode the stream
  if (!pb_decode(&stream,
                 wippersnapper_description_v1_CreateDescriptionResponse_fields,
                 &message)) {
    WS_DEBUG_PRINTLN("Error decoding description status message!");
  } else { // set board status
    switch (message.response) {
    case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK:
      WS_DEBUG_PRINTLN("Found hardware with:")
      WS_DEBUG_PRINT("GPIO Pins: ");
      WS_DEBUG_PRINTLN(message.total_gpio_pins);
      WS_DEBUG_PRINT("Analog Pins: ");
      WS_DEBUG_PRINTLN(message.total_analog_pins);
      WS_DEBUG_PRINT("Reference voltage: ");
      WS_DEBUG_PRINT(message.reference_voltage);
      WS_DEBUG_PRINTLN("v");
      // Initialize Digital IO class
      WS._digitalGPIO = new Wippersnapper_DigitalGPIO(message.total_gpio_pins);
      // Initialize Analog IO class
      WS._analogIO = new Wippersnapper_AnalogIO(message.total_analog_pins,
                                                message.reference_voltage);
      WS._boardStatus = WS_BOARD_DEF_OK;
      break;
    case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND:
      WS._boardStatus = WS_BOARD_DEF_INVALID;
      break;
    case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED:
      WS._boardStatus = WS_BOARD_DEF_UNSPECIFIED;
      break;
    default:
      WS._boardStatus = WS_BOARD_DEF_UNSPECIFIED;
    }
  }
}