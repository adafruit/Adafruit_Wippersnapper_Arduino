/*!
 * @file src/components/telemetry/model.h
 *
 * Model interface for the telemetry.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_TELEMETRY_MODEL_H
#define WS_TELEMETRY_MODEL_H
#include "wippersnapper.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from telemetry.proto.
*/
class TelemetryModel {
public:
  TelemetryModel();
  ~TelemetryModel();
  // TelemetryAdd Message API
  bool DecodeTelemetryAdd(pb_istream_t *stream);
  ws_telemetry_Add *GetTelemetryAddMsg();
  // TelemetryRemove Message API
  bool DecodeTelemetryRemove(pb_istream_t *stream);
  ws_telemetry_Remove *GetTelemetryRemoveMsg();
  // TelemetryEvent Message API
  void InitEventMsg(ws_telemetry_Type type);
  void SetValueFloat(ws_sensor_Type type, float value);
  void SetValueString(ws_sensor_Type type, const char *value);
  bool EncodeEvent();
  ws_telemetry_D2B *GetD2B();

private:
  static bool encode_string_cb(pb_ostream_t *stream, const pb_field_t *field,
                               void *const *arg);
  ws_telemetry_Add _msg_Add;       ///< Add message
  ws_telemetry_Remove _msg_Remove; ///< Remove message
  ws_telemetry_D2B _msg_D2B;       ///< D2B wrapper holding the Event
  char _str_value[96]; ///< Backing store for a string value (bytes_value)
};
#endif // WS_TELEMETRY_MODEL_H
