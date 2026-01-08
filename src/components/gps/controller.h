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
#include "hardware.h"
#include "model.h"
#include "wippersnapper.h"
#include <Adafruit_GPS.h>

class wippersnapper; ///< Forward declaration
class GPSModel;      ///< Forward declaration
class GPSHardware;   ///< Forward declaration

/*!
    @brief  Routes messages between the GPS.proto API and the hardware.
*/
class GPSController {
public:
  GPSController();
  ~GPSController();
  bool AddGPS(HardwareSerial *serial, ws_gps_Config *gps_config);
  bool AddGPS(TwoWire *wire, uint32_t i2c_addr, ws_gps_Config *gps_config);
  void update(bool force_read_all = false);
  bool UpdateComplete();

private:
  GPSModel *_gps_model;                    ///< GPS model instance
  std::vector<GPSHardware *> _gps_drivers; ///< GPS hardware instances
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_GPS_CONTROLLER_H