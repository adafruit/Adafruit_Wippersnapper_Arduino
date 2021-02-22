/*!
 * @file Wippersnapper_Checkin.h
 *
 * This file provides protocol buffer decoders for the Wippersnapper
 * protocol API.
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
#ifndef WIPPERSNAPPER_REGISTRATION_H
#define WIPPERSNAPPER_REGISTRATION_H

#include <Wippersnapper.h>

// forward declaration
class Wippersnapper;

class Wippersnapper_Registration {
    public:
        Wippersnapper_Registration(Wippersnapper *ws);
        ~Wippersnapper_Registration();

        bool processRegistration();
        void encodeRegMsg();
        void publishRegMsg();
        bool pollRegMsg();
        void decodeRegMsg(char *data, uint16_t len);

    private:

        enum class FSMReg {
            REG_CREATE_ENCODE_MSG,
            REG_PUBLISH_MSG,
            REG_DECODE_MSG,
            REG_DECODED_MSG,
        };
        FSMReg _state = FSMReg::REG_CREATE_ENCODE_MSG;

        uint8_t _message_buffer[128];
        size_t _message_len;
        bool _status;

        pb_ostream_t _msg_stream;

        // struct for description msg.
        //wippersnapper_description_v1_CreateDescriptionRequest _message;

        // Description message contents
        const char * _machine_name;
        int32_t _uid;

        Wippersnapper *_ws; // instance of Wippersnapper
};

#endif // WIPPERSNAPPER_REGISTRATION_H