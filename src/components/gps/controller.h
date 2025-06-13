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

class Wippersnapper_V2; ///< Forward declaration
class GPSModel;        ///< Forward declaration
class GPSHardware;     ///< Forward declaration

/*!
    @brief  Routes messages between the GPS.proto API and the hardware.
*/
class GPSController {
public:
  GPSController();
  ~GPSController();
  bool Handle_GPSConfig(pb_istream_t *stream);
  bool RemoveGPSDevice(const char *id);
  void update();
private:
  GPSModel *_gps_model;                     ///< GPS model
  std::vector<GPSHardware *> _gps_devices;  ///< Vector of GPS hardware instances
  std::vector<drvGpsBase *> _gps_drivers;   ///< Vector of GPS device drivers
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif // WS_GPS_CONTROLLER_H