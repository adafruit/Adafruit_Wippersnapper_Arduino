/*!
 * @file src/components/analogIO/model.h
 *
 * Model interface for the analogin.proto message.
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
#include "wippersnapper.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from analogin.proto.
*/
class AnalogIOModel {
public:
  AnalogIOModel();
  ~AnalogIOModel();
  // AnalogInAdd
  bool decodeAnalogInAdd(pb_istream_t *stream);
  ws_analogin_Add *getAnalogInAddMsg();
  // AnalogInRemove
  bool decodeAnalogInRemove(pb_istream_t *stream);
  ws_analogin_Remove *GetAnalogInRemoveMsg();
  // AnalogInEvent
  bool encodeAnalogInEvent(const char *pin_name, float pin_value,
                           ws_sensor_Type read_type);
  bool encodeAnalogInEventVoltage(const char *pin_name, float pin_value);
  bool encodeAnalogInEventRaw(const char *pin_name, float pin_value);
  ws_analogin_Event *getAnalogInEvent();
  ws_analogin_D2B *getAnalogInD2b();

private:
  ws_analogin_Add _msg_AnalogInAdd;       ///< AnalogInAdd message
  ws_analogin_Remove _msg_AnalogInRemove; ///< AnalogInRemove message
  ws_analogin_Event _msg_AnalogInEvent;   ///< AnalogInEvent message
  ws_analogin_D2B _msg_AnalogInD2B;       ///< AnalogIn DeviceToBroker wrapper
};
#endif // WS_ANALOGIO_MODEL_H
