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

/****************************************************************************/
/*!
    @brief    Encodes hardware registration request message and publishes
              the message to the Adafruit IO broker.
    @returns  True if encoded and/or published successfully, False otherwise.
*/
/****************************************************************************/
bool Wippersnapper::encodePubRegistrationReq() {
  bool _status;

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
  uint8_t _message_buffer[256];
  pb_ostream_t _msg_stream =
      pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

  _status = pb_encode(
      &_msg_stream,
      wippersnapper_description_v1_CreateDescriptionRequest_fields, &_message);
  size_t _message_len = _msg_stream.bytes_written;

  // verify message
  if (!_status)
    return _status;

  // pubish message
  WS.publish(WS._topic_description, _message_buffer, _message_len, 1);
  WS_DEBUG_PRINTLN("Published!");
  WS._boardStatus = WS_BOARD_DEF_SENT;

  return _status;
}

/****************************************************************************/
/*!
    @brief    Polls the broker for the hardware registration response message.

              NOTE: This function is BLOCKING and will trigger a WDT reset
              if the message has not arrived.

             NOTE: The registration response msg will arrive
             async. at the `cbRegistrationStatus` function
             and set the `boardStatus`
*/
/****************************************************************************/
void Wippersnapper::pollRegistrationResp() {
  // Blocking loop, WDT reset upon failure.
  while (WS._boardStatus != WS_BOARD_DEF_OK) {
    WS_DEBUG_PRINT("Polling for registration message response...");
    WS_DEBUG_PRINTLN(WS._boardStatus);
    statusLEDBlink(WS_LED_STATUS_WAITING_FOR_REG_MSG);
    WS._mqtt->processPackets(20); // long-poll
  }
}

/****************************************************************************/
/*!
    @brief    Decodes hardware registration response message from the
              Adafruit IO MQTT broker and initializes hardware components.
    @param    data
              MQTT message from the Adafruit IO MQTT broker.
    @param    len
              Length of data from the Adafruit IO MQTT broker.
*/
/****************************************************************************/
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
  if (message.response ==
      wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK) {
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

    // Publish RegistrationComplete message to broker
    wippersnapper_description_v1_RegistrationComplete msg =
        wippersnapper_description_v1_RegistrationComplete_init_zero;
    msg.is_complete = true;

    // encode registration request message
    uint8_t _message_buffer[128];
    pb_ostream_t _msg_stream =
        pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

    bool _status = pb_encode(
        &_msg_stream, wippersnapper_description_v1_RegistrationComplete_fields,
        &msg);
    size_t _message_len = _msg_stream.bytes_written;

    // verify message encoded correctly
    if (!_status) {
      WS._boardStatus = WS_BOARD_DEF_INVALID;
      return;
    }

    // Publish message
    WS.publish(_topic_description_status_complete, _message_buffer,
               _message_len, 1);
    WS_DEBUG_PRINTLN("Completed registration process, configuration next!");

  } else {
    WS._boardStatus = WS_BOARD_DEF_INVALID;
  }
}