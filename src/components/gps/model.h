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
#define MAX_COUNT_RMC_GGA 10 ///< Maximum number of RMC or GGA responses

class GPSHardware; ///< Forward declaration

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from GPS.proto.
*/
class GPSModel {
public:
  GPSModel();
  ~GPSModel();
  bool DecodeGPSConfig(pb_istream_t *stream);
  ws_gps_Config *GetGPSConfigMsg();
  // GPSEvent API
  void CreateGPSEvent();
  bool EncodeGPSEvent();
  ws_gps_Event *GetGPSEvent();
  bool ProcessNMEASentence(char *sentence, GPSHardware *drv);
  ws_gps_DateTime CreateGpsDatetime(uint8_t hour, uint8_t minute,
                                    uint8_t seconds, uint8_t milliseconds,
                                    uint8_t day, uint8_t month, uint8_t year);
  bool AddGpsEventRMC(ws_gps_DateTime datetime, uint8_t fix_status, float lat,
                      char *lat_dir, float lon, char *lon_dir, float speed,
                      float angle);

  bool AddGpsEventGGA(ws_gps_DateTime datetime, uint8_t fix_status, float lat,
                      char *lat_dir, float lon, char *lon_dir, uint8_t num_sats,
                      float hdop, float alt, float geoid_height);

private:
  ws_gps_Config _msg_gps_config; ///< GPS configuration message
  ws_gps_Event _msg_gps_event;   ///< GPS event message
};
#endif // WS_GPS_MODEL_H