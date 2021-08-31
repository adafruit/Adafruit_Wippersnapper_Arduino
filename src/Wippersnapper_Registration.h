/*!
 * @file Wippersnapper_Registration.h
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_REGISTRATION_H
#define WIPPERSNAPPER_REGISTRATION_H

#include "Wippersnapper.h"

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that negotiates the device's registration process with
            the Adafruit IO MQTT broker.
*/
/**************************************************************************/
class Wippersnapper_Registration {
public:
  Wippersnapper_Registration();
  ~Wippersnapper_Registration();

  bool processRegistration();
  void encodeRegMsg();
  void publishRegMsg();
  void pollRegMsg();
  void decodeRegMsg(char *data, uint16_t len);

private:
  enum class FSMReg {
    REG_CREATE_ENCODE_MSG,
    REG_PUBLISH_REQ_MSG,
    REG_POLL_MSG,
    REG_PARSE_RESP_MSG,
  };
  FSMReg _state = FSMReg::REG_CREATE_ENCODE_MSG;

  uint8_t _message_buffer[128];
  size_t _message_len;
  bool _status;

  pb_ostream_t _msg_stream;

  // Description message contents
  const char *_machine_name;
  int32_t _uid;
};

extern Wippersnapper WS;

#endif // WIPPERSNAPPER_REGISTRATION_H