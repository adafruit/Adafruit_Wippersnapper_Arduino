/*!
 * @file model.h
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

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from DS18X20.proto.
*/
/**************************************************************************/
class DS18X20Model {
public:
  DS18X20Model();
  ~DS18X20Model();
  // Ds18x20Add Message
  bool DecodeDS18x20Add(pb_istream_t *stream);
  wippersnapper_ds18x20_Ds18x20Add *GetDS18x20AddMsg();

private:
  wippersnapper_ds18x20_Ds18x20Add
      _msg_DS18x20Add; ///< wippersnapper_ds18x20_Ds18x20Add message
};
#endif // WS_DIGITALIO_MODEL_H