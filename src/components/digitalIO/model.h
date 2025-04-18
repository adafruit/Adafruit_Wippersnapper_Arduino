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

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from digitalio.proto.
*/
/**************************************************************************/
class DigitalIOModel {
public:
  DigitalIOModel();
  ~DigitalIOModel();
  // DigitalIOAdd
  bool DecodeDigitalIOAdd(pb_istream_t *stream);
  wippersnapper_digitalio_DigitalIOAdd *GetDigitalIOAddMsg();
  // DigitalIORemove
  bool DecodeDigitalIORemove(pb_istream_t *stream);
  // DigitalIOWrite
  bool DecodeDigitalIOWrite(pb_istream_t *stream);
  wippersnapper_digitalio_DigitalIOWrite *GetDigitalIOWriteMsg();
  // DigitalIOEvent
  bool EncodeDigitalIOEvent(char *pin_name, bool value);
  wippersnapper_digitalio_DigitalIOEvent *GetDigitalIOEventMsg();

private:
  wippersnapper_digitalio_DigitalIOAdd
      _msg_dio_add; ///< DigitalIOAdd message object
  wippersnapper_digitalio_DigitalIORemove
      _msg_dio_remove; ///< DigitalIORemove message object
  wippersnapper_digitalio_DigitalIOEvent
      _msg_dio_event; ///< DigitalIOEvent message object
  wippersnapper_digitalio_DigitalIOWrite
      _msg_dio_write; ///< DigitalIOWrite message object
};
#endif // WS_DIGITALIO_MODEL_H