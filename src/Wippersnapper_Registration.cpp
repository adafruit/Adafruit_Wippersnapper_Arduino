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

Wippersnapper_Registration::Wippersnapper_Registration(const char *machine_name, int32_t uid) {
    _machine_name = machine_name;
    _uid = uid;
}

/**************************************************************************/
/*!
    @brief    Creates new board description request message containing
                device-specific information.
*/
/**************************************************************************/
void Wippersnapper_Registration::set_description() {
    wippersnapper_description_v1_CreateDescriptionRequest _message = wippersnapper_description_v1_CreateDescriptionRequest_init_zero;
    pb_ostream_t _msg_stream = pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

    // fill message fields
    strcpy(message.machine_name, _machine_name);
    message.mac_addr = _uid;

    // encode message fields
}

/**************************************************************************/
/*!
    @brief    Publishes description message to Wippersnapper.
*/
/**************************************************************************/
bool Wippersnapper_Registration::publish_description() {
    WS_DEBUG_PRINT("Publishing description message...");
    _mqtt->publish(_topic_description, _message_buffer, _message_len, 0);

    WS_DEBUG_PRINTLN("Published board description, waiting for response!");
    _boardStatus = WS_BOARD_DEF_SENT;
    return true; // TODO: Catch failures from _mqtt->publish() calls!
}