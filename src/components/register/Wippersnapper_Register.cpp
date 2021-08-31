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

bool Wippersnapper::encodePubRegistrationReq() {
  bool _status;
  uint8_t _message_buffer[128];
  size_t _message_len;
  pb_ostream_t _msg_stream;

  WS_DEBUG_PRINT("Encoding registration msg...");
  // Create message object
  wippersnapper_description_v1_CreateDescriptionRequest _message =
      wippersnapper_description_v1_CreateDescriptionRequest_init_zero;

  // Set machine_name
  strcpy(_message.machine_name, WS._boardId);

  // Set MAC address
  _message.mac_addr = atoi(WS.sUID);

  // Set version
  strcpy(_message.str_version, WS_VERSION);

  // encode registration request message
  _msg_stream = pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

  _status = pb_encode(
      &_msg_stream,
      wippersnapper_description_v1_CreateDescriptionRequest_fields, &_message);
  _message_len = _msg_stream.bytes_written;

  // verify message
  if (!_status)
    return _status;

  // pubish message
  WS.publish(WS._topic_description, _message_buffer, _message_len, 1);
  WS_DEBUG_PRINTLN("Published!");
  WS._boardStatus = WS_BOARD_DEF_SENT;

  return _status;
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
    WS_DEBUG_PRINTLN("ERROR: Did not get registration message from broker, polling...");
    WS.setStatusLEDColor(LED_ERROR);
  }
}

void Wippersnapper::decodeRegistrationResp(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("GOT Registration Response Message:");
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
    WS.haltError("Could not decode registration response");
  }
  // Decode registration response message
  if (message.response == wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK) {
      WS_DEBUG_PRINTLN("Hardware Response Msg:")
      WS_DEBUG_PRINT("\tGPIO Pins: ");
      WS_DEBUG_PRINTLN(message.total_gpio_pins);
      WS_DEBUG_PRINT("\tAnalog Pins: ");
      WS_DEBUG_PRINTLN(message.total_analog_pins);
      WS_DEBUG_PRINT("\tReference voltage: ");
      WS_DEBUG_PRINT(message.reference_voltage);
      WS_DEBUG_PRINTLN("v");
      // Initialize Digital IO class
      WS._digitalGPIO = new Wippersnapper_DigitalGPIO(message.total_gpio_pins);
      // Initialize Analog IO class
      WS._analogIO = new Wippersnapper_AnalogIO(message.total_analog_pins,
                                                message.reference_voltage);
      WS._boardStatus = WS_BOARD_DEF_OK;
  } else {
    WS._boardStatus = WS_BOARD_DEF_INVALID;
  }
}