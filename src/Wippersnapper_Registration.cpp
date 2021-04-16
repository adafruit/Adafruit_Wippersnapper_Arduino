/*!
 * @file Wippersnapper_Checkin.cpp
 *
 * This file provides an API for registering a
 * device with the Wippersnapper MQTT broker.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper_Registration.h"

/**************************************************************************/
/*!
    @brief    Creates a new instance of a registration object.
    @param    *ws
              Reference to Wippersnapper.
*/
/**************************************************************************/
Wippersnapper_Registration::Wippersnapper_Registration() {
    //wippersnapper_description_v1_CreateDescriptionRequest _message = wippersnapper_description_v1_CreateDescriptionRequest_init_zero;

}

/************************************************************/
/*!
    @brief    Registration destructor
*/
/************************************************************/
Wippersnapper_Registration::~Wippersnapper_Registration() {
    if (_message_buffer)
        free(_message_buffer);

    if (_machine_name)
        free((char*)_machine_name);

    if (_uid)
        _uid = 0;

    // reset FSM
    _state = FSMReg::REG_CREATE_ENCODE_MSG;
}

/************************************************************/
/*!
    @brief    Registration State Machine. Handles the
                hardware registration process.
    @returns True if registered with Wippersnapper 
                successfully, False otherwise.
*/
/************************************************************/
bool Wippersnapper_Registration::processRegistration() {
    bool is_registered = false;
    FSMReg next_state = _state;

    switch(_state) {
        case FSMReg::REG_CREATE_ENCODE_MSG:
            WS_DEBUG_PRINT("Encoding registration message...");
            encodeRegMsg();
            next_state = FSMReg::REG_PUBLISH_MSG;
        case FSMReg::REG_PUBLISH_MSG:
            WS_DEBUG_PRINT("Publishing registration message...");
            publishRegMsg();
            next_state = FSMReg::REG_DECODE_MSG;
        case FSMReg::REG_DECODE_MSG:
            if (!pollRegMsg()) {
                next_state = FSMReg::REG_CREATE_ENCODE_MSG;
            } else {
                next_state = FSMReg::REG_DECODED_MSG;
            }
        case FSMReg::REG_DECODED_MSG:
            is_registered = true; // if successful
            break;
        default:
            break;
    }
    return is_registered;
}


/************************************************************/
/*!
    @brief    Creates and encodes a registration protobuf
                message.
*/
/************************************************************/
void Wippersnapper_Registration::encodeRegMsg() {
    _status = true;

    // Create message object
    wippersnapper_description_v1_CreateDescriptionRequest _message = wippersnapper_description_v1_CreateDescriptionRequest_init_zero;
    
    // Fill message object's fields
    _machine_name = WS._boardId;
    _uid = atoi(WS.sUID);

    // Encode fields
    strcpy(_message.machine_name, _machine_name);
    _message.mac_addr = _uid;

    pb_ostream_t _msg_stream = pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

    // encode message
    _status = pb_encode(&_msg_stream, wippersnapper_description_v1_CreateDescriptionRequest_fields, &_message);
    _message_len = _msg_stream.bytes_written;

    // verify message
    if (!_status) {
        WS_DEBUG_PRINTLN("ERROR: encoding description message failed!");
        //printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    WS_DEBUG_PRINTLN("Encoded!");
}

/************************************************************/
/*!
    @brief    Publishes a registration message to the
                Wippersnapper broker.
*/
/************************************************************/
void Wippersnapper_Registration::publishRegMsg() {
    WS._mqtt->publish(WS._topic_description, _message_buffer, _message_len, 1);
    WS_DEBUG_PRINTLN("Published!")
    WS._boardStatus = WS_BOARD_DEF_SENT;
}

/************************************************************/
/*!
    @brief    Polls the broker for a response to a published
                registration message.
*/
/************************************************************/
bool Wippersnapper_Registration::pollRegMsg() {
    bool is_success = false;
    // Attempt to obtain response from broker
    uint8_t retryCount = 0;
    while (WS._boardStatus == WS_BOARD_DEF_SENT) {
        if (retryCount >= 10) {
            WS_DEBUG_PRINTLN("Exceeded retries, failing out...");
            break;
        }
        WS._mqtt->processPackets(500); // pump MQTT msg loop
        retryCount++;
    }
    return is_success;
}

/************************************************************/
/*!
    @brief    Decodes registration message from broker.
*/
/************************************************************/
void Wippersnapper_Registration::decodeRegMsg(char *data, uint16_t len) {
    WS_DEBUG_PRINTLN("-> Registration Message");
    uint8_t buffer[len];
    memcpy(buffer, data, len);

    // init. CreateDescriptionResponse message
    wippersnapper_description_v1_CreateDescriptionResponse message = wippersnapper_description_v1_CreateDescriptionResponse_init_zero;

    // create input stream for buffer
    pb_istream_t stream = pb_istream_from_buffer(buffer, len);
    // decode the stream
    if (!pb_decode(&stream, wippersnapper_description_v1_CreateDescriptionResponse_fields, &message)) {
        WS_DEBUG_PRINTLN("Error decoding description status message!");
    } else {    // set board status
        switch (message.response) {
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK:
                WS._boardStatus = WS_BOARD_DEF_OK;
                // Fetch information about hardware
                WS.totalDigitalPins = message.total_gpio_pins;
                WS.totalAnalogPins = message.total_analog_pins;
                WS.vRef = message.reference_voltage;
                WS_DEBUG_PRINTLN("Found hardware with:")
                WS_DEBUG_PRINT("GPIO Pins: "); WS_DEBUG_PRINTLN(WS.totalDigitalPins);
                WS_DEBUG_PRINT("Analog Pins: "); WS_DEBUG_PRINTLN(WS.totalAnalogPins);
                WS_DEBUG_PRINT("Reference voltage: "); WS_DEBUG_PRINT(WS.vRef); WS_DEBUG_PRINTLN("v");
                break;
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND:
                //_ws->_boardStatus = WS_BOARD_DEF_INVALID;
                WS._boardStatus = WS_BOARD_DEF_INVALID;
                break;
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED:
                //_ws->_boardStatus = WS_BOARD_DEF_UNSPECIFIED;
                WS._boardStatus = WS_BOARD_DEF_UNSPECIFIED;
                break;
            default:
                WS._boardStatus = WS_BOARD_DEF_UNSPECIFIED;
        }
    }

    WS_DEBUG_PRINTLN("\nSuccessfully checked in, waiting for commands...");
}