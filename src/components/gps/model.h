/*!
 * @file src/components/gps/model.h
 *
 * Model interface for the GPS.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_GPS_MODEL_H
#define WS_GPS_MODEL_H
#include "Wippersnapper_V2.h"
#include <protos/gps.pb.h>

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from GPS.proto.
*/
class GPSModel {
public:
  GPSModel();
  ~GPSModel();
  bool DecodeGPSConfig(pb_istream_t *stream);
  wippersnapper_gps_GPSConfig *GetGPSConfigMsg();
private:
  wippersnapper_gps_GPSConfig _msg_gps_config;           ///< GPS configuration message
};
#endif // WS_GPS_MODEL_H