/*!
 * @file src/components/analog/model.h
 *
 * Model interface for the analog.proto message.
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
#ifndef WS_ANALOG_MODEL_H
#define WS_ANALOG_MODEL_H
#include "wippersnapper.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from analog.proto.
*/
class AnalogModel {
public:
  AnalogModel();
  ~AnalogModel();
  // AnalogAdd
  bool decodeAnalogAdd(pb_istream_t *stream);
  ws_analog_Add *getAnalogAddMsg();
  // AnalogRemove
  bool decodeAnalogRemove(pb_istream_t *stream);
  ws_analog_Remove *GetAnalogRemoveMsg();
  // AnalogEvent
  bool encodeAnalogEvent(const char *pin_name, float pin_value,
                         ws_sensor_Type read_type);
  bool encodeAnalogEventVoltage(const char *pin_name, float pin_value);
  bool encodeAnalogEventRaw(const char *pin_name, float pin_value);
  ws_analog_Event *getAnalogEvent();
  ws_analog_D2B *getAnalogD2b();

private:
  ws_analog_Add _msg_AnalogAdd;       ///< AnalogAdd message
  ws_analog_Remove _msg_AnalogRemove; ///< AnalogRemove message
  ws_analog_Event _msg_AnalogEvent;   ///< AnalogEvent message
  ws_analog_D2B _msg_AnalogD2B;       ///< Analog DeviceToBroker wrapper
};
#endif // WS_ANALOG_MODEL_H
