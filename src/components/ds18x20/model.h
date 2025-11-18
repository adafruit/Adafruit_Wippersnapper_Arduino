/*!
 * @file src/components/ds18x20/model.h
 *
 * Model interface for the DS18X20.proto message.
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
#ifndef WS_DS18X20_MODEL_H
#define WS_DS18X20_MODEL_H
#include "Wippersnapper_V2.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from DS18X20.proto.
*/
class DS18X20Model {
public:
  DS18X20Model();
  ~DS18X20Model();
  // Ds18x20Add Message API
  bool DecodeDS18x20Add(pb_istream_t *stream);
  ws_ds18x20_Add *GetDS18x20AddMsg();
  // DS18x20Added Message API
  bool EncodeDS18x20Added(char *onewire_pin, bool is_init);
  ws_ds18x20_Added *GetDS18x20AddedMsg();
  // Ds18x20Remove Message API
  bool DecodeDS18x20Remove(pb_istream_t *stream);
  ws_ds18x20_Remove *GetDS18x20RemoveMsg();
  // Ds18x20Event Message API
  bool EncodeDs18x20Event();
  ws_ds18x20_Event *GetDS18x20EventMsg();
  void InitDS18x20EventMsg(const char *ow_pin_name);
  void addSensorEvent(ws_sensor_Type sensor_type,
                      float sensor_value);

private:
  ws_ds18x20_Add _msg_DS18x20Add;       ///< Add message
  ws_ds18x20_Added _msg_DS18x20Added;   ///< Added message
  ws_ds18x20_Remove _msg_DS18x20Remove; ///< Remove message
  ws_ds18x20_Event _msg_DS18x20Event;   ///< Event message
};
#endif // WS_DIGITALIO_MODEL_H