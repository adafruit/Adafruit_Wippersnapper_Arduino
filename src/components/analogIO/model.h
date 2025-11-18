/*!
 * @file src/components/analogIO/model.h
 *
 * Model interface for the analogio.proto message.
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
#ifndef WS_ANALOGIO_MODEL_H
#define WS_ANALOGIO_MODEL_H
#include "Wippersnapper_V2.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from analogio.proto.
*/
class AnalogIOModel {
public:
  AnalogIOModel();
  ~AnalogIOModel();
  // AnalogIOAdd
  bool DecodeAnalogIOAdd(pb_istream_t *stream);
  ws_analogio_Add *GetAnalogIOAddMsg();
  // AnalogIORemove
  bool DecodeAnalogIORemove(pb_istream_t *stream);
  ws_analogio_Remove *GetAnalogIORemoveMsg();
  // AnalogIOEvent
  bool EncodeAnalogIOEvent(char *pin_name, float pin_value,
                           ws_sensor_Type read_type);
  bool EncodeAnalogIOEventVoltage(char *pin_name, float pin_value);
  bool EncodeAnalogIOEventRaw(char *pin_name, float pin_value);
  ws_analogio_Event *GetAnalogIOEvent();

private:
  ws_analogio_Add _msg_AnalogioAdd;       ///< AnalogIOAdd message
  ws_analogio_Remove _msg_AnalogioRemove; ///< AnalogIORemove message
  ws_analogio_Event _msg_AnalogioEvent;   ///< AnalogIOEvent message
};
#endif // WS_DIGITALIO_MODEL_H