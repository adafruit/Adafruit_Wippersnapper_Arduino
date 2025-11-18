/*!
 * @file src/components/digitalIO/model.h
 *
 * Model for the digitalio.proto message.
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
#ifndef WS_DIGITALIO_MODEL_H
#define WS_DIGITALIO_MODEL_H
#include "Wippersnapper_V2.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from digitalio.proto.
*/
class DigitalIOModel {
public:
  DigitalIOModel();
  ~DigitalIOModel();
  // DigitalIOAdd
  bool DecodeDigitalIOAdd(pb_istream_t *stream);
  ws_digitalio_Add *GetDigitalIOAddMsg();
  // DigitalIORemove
  bool DecodeDigitalIORemove(pb_istream_t *stream);
  // DigitalIOWrite
  bool DecodeDigitalIOWrite(pb_istream_t *stream);
  ws_digitalio_Write *GetDigitalIOWriteMsg();
  // DigitalIOEvent
  bool EncodeDigitalIOEvent(char *pin_name, bool value);
  ws_digitalio_Event *GetDigitalIOEventMsg();

private:
  ws_digitalio_Add _msg_dio_add;       ///< Add message object
  ws_digitalio_Remove _msg_dio_remove; ///< Remove message object
  ws_digitalio_Event _msg_dio_event;   ///< Event message object
  ws_digitalio_Write _msg_dio_write;   ///< Write message object
};
#endif // WS_DIGITALIO_MODEL_H