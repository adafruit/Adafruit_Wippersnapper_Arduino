/*!
 * @file src/components/digital/model.h
 *
 * Model for the digital.proto message.
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
#ifndef WS_DIGITAL_MODEL_H
#define WS_DIGITAL_MODEL_H
#include "wippersnapper.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from digital.proto.
*/
class DigitalModel {
public:
  DigitalModel();
  ~DigitalModel();
  // DigitalAdd
  bool DecodeDigitalAdd(pb_istream_t *stream);
  ws_digital_Add *GetDigitalAddMsg();
  // DigitalRemove
  bool DecodeDigitalRemove(pb_istream_t *stream);
  // DigitalWrite
  bool DecodeDigitalWrite(pb_istream_t *stream);
  ws_digital_Write *GetDigitalWriteMsg();
  // DigitalEvent
  bool EncodeDigitalEvent(char *pin_name, bool value);
  ws_digital_Event *GetDigitalEventMsg();
  ws_digital_D2B *GetDigitalD2B();

private:
  ws_digital_Add _msg_digital_add;       ///< Add message object
  ws_digital_Remove _msg_digital_remove; ///< Remove message object
  ws_digital_Event _msg_digital_event;   ///< Event message object
  ws_digital_Write _msg_digital_write;   ///< Write message object
  ws_digital_D2B _msg_digital_d2b;       ///< Digital DeviceToBroker wrapper
};
#endif // WS_DIGITAL_MODEL_H