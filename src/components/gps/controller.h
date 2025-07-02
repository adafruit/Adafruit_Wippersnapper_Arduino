/*!
 * @file src/components/gps/controller.h
 *
 * Controller for WipperSnapper's GPS component, bridges between the GPS.proto
 * API, the model, and the hardware layer.
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
#ifndef WS_GPS_CONTROLLER_H
#define WS_GPS_CONTROLLER_H
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"
#include <Adafruit_GPS.h>

#define MAX_NMEA_SENTENCES 10    ///< Size of the NMEA buffer
#define MAX_LEN_NMEA_SENTENCE 82 ///< Maximum length of a NMEA sentence

typedef struct {
  char sentences[MAX_NMEA_SENTENCES][MAX_LEN_NMEA_SENTENCE];
  int head;
  int tail;
  int maxlen;
} nmea_buffer_t;

class Wippersnapper_V2; ///< Forward declaration
class GPSModel;         ///< Forward declaration
class GPSHardware;      ///< Forward declaration

/*!
    @brief  Routes messages between the GPS.proto API and the hardware.
*/
class GPSController {
public:
  GPSController();
  ~GPSController();
  bool AddGPS(HardwareSerial *serial, wippersnapper_gps_GPSConfig *gps_config);
  bool AddGPS(TwoWire *wire, uint32_t i2c_addr,
              wippersnapper_gps_GPSConfig *gps_config);
  void update();

private:
  int NmeaBufPush(const char *new_sentence);
  int NmeaBufPop(char *sentence);
  GPSModel *_gps_model;                    ///< GPS model instance
  std::vector<GPSHardware *> _gps_drivers; ///< GPS hardware instances
  nmea_buffer_t _nmea_buff; ///< NMEA buffer for storing sentences
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_GPS_CONTROLLER_H