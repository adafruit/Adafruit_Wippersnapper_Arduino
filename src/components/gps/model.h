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
#define MAX_COUNT_RMC_GGA 10; ///< Maximum number of RMC or GGA responses

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
  // GPSEvent API
  void CreateGPSEvent();
  wippersnapper_gps_GPSDateTime CreateGpsDatetime(uint8_t hour, uint8_t minute,
                                                  uint8_t seconds,
                                                  uint8_t milliseconds,
                                                  uint8_t day, uint8_t month,
                                                  uint8_t year);
  bool AddGpsEventRMC(wippersnapper_gps_GPSDateTime *datetime,
                      uint8_t fix_status, nmea_float_t latitude, char *lat_dir,
                      nmea_float_t longitude, char *lon_dir, nmea_float_t speed,
                      nmea_float_t angle);

private:
  wippersnapper_gps_GPSConfig _msg_gps_config; ///< GPS configuration message
  wippersnapper_gps_GPSEvent _msg_gps_event;   ///< GPS event message
};
#endif // WS_GPS_MODEL_H