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
Wippersnapper_Registration::Wippersnapper_Registration(Wippersnapper *ws) {
    _ws = ws;
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
}

bool Wippersnapper_Registration::process_registration() {
    bool is_registered = false;
    FSMReg next_state = _state;

    switch(_state) {
        case FSMReg::REG_CREATE_MSG:
            WS_DEBUG_PRINTLN("Creating registration message");
            next_state = FSMReg::REG_ENCODE_MSG;
        case FSMReg::REG_ENCODE_MSG:
            WS_DEBUG_PRINTLN("Encoding registration message");
            next_state = FSMReg::REG_PUBLISH_MSG;
        case FSMReg::REG_PUBLISH_MSG:
            WS_DEBUG_PRINTLN("Publishing registration message");
            next_state = FSMReg::REG_DECODE_MSG;
        case FSMReg::REG_DECODE_MSG:
            WS_DEBUG_PRINTLN("Decoding registration message");
            is_registered = true; // if successful
            break;
        default:
            break;
    }
    return is_registered;
}


/**************************************************************************/
/*!
    @brief    Sets the message's machine_name field.
    @param    machine_name
              Valid machine name.
*/
/**************************************************************************/
void Wippersnapper_Registration::setMachineName(const char *machine_name) {
    _machine_name = machine_name;
    //strcpy(_message.machine_name, _machine_name);

}

/**************************************************************************/
/*!
    @brief    Sets the message's mac_addr field.
    @param    uid
              Valid unique identifier.
*/
/**************************************************************************/
void Wippersnapper_Registration::setUID(int32_t uid) {
    _uid = uid;
    //_message.mac_addr = _uid;
}

/**************************************************************************/
/*!
    @brief    Encodes a CreateDescriptionRequest message.
*/
/**************************************************************************/
bool Wippersnapper_Registration::encodeDescRequest() {
    WS_DEBUG_PRINTLN("encoding board description...");
    _status = true;

    wippersnapper_description_v1_CreateDescriptionRequest _message = wippersnapper_description_v1_CreateDescriptionRequest_init_zero;

    strcpy(_message.machine_name, _machine_name);
    _message.mac_addr = _uid;

    pb_ostream_t _msg_stream = pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

    // encode message
    _status = pb_encode(&_msg_stream, wippersnapper_description_v1_CreateDescriptionRequest_fields, &_message);
    _message_len = _msg_stream.bytes_written;

    // verify message
    if (!_status) {
        WS_DEBUG_PRINTLN("encoding description message failed!");
        //printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    return _status;
}

/**************************************************************************/
/*!
    @brief    Publishes description message to Wippersnapper.
*/
/**************************************************************************/
void Wippersnapper_Registration::publishDescRequest() {
    WS_DEBUG_PRINT("Publishing description message...");
    if (!_ws->_mqtt->publish(_ws->_topic_description, _message_buffer, _message_len, 0)) {
        WS_DEBUG_PRINTLN("Board registration message failed to publish to Wippersnapper.")
        _ws->_boardStatus = WS_BOARD_DEF_SEND_FAILED;
    }
    _ws->_boardStatus = WS_BOARD_DEF_SENT;
}

/**************************************************************************/
/*!
    @brief    Decodes description response from broker.
*/
/**************************************************************************/
void Wippersnapper_Registration::decodeDescResponse(uint8_t buffer, uint16_t len) {
    WS_DEBUG_PRINTLN("Decoding description response from broker");
    // init. CreateDescriptionResponse message
    wippersnapper_description_v1_CreateDescriptionResponse message = wippersnapper_description_v1_CreateDescriptionResponse_init_zero;

    // create input stream for buffer
    pb_istream_t stream = pb_istream_from_buffer(&buffer, len);
    // decode the stream
    if (!pb_decode(&stream, wippersnapper_description_v1_CreateDescriptionResponse_fields, &message)) {
        WS_DEBUG_PRINTLN("Error decoding description status message!");
        _ws->_boardStatus = WS_BOARD_DEF_UNSPECIFIED;
    } else {    // set board status
        switch (message.response) {
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK:
                _ws->_boardStatus = WS_BOARD_DEF_OK;
                break;
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND:
                _ws->_boardStatus = WS_BOARD_DEF_INVALID;
                break;
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED:
                _ws->_boardStatus = WS_BOARD_DEF_UNSPECIFIED;
                break;
            default:
                _ws->_boardStatus = WS_BOARD_DEF_UNSPECIFIED;
        }
    }

}