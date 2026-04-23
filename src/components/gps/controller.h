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
#include "components/uart/hardware.h"
#include "hardware.h"
#include "model.h"
#include "wippersnapper.h"
#include <Adafruit_GPS.h>

class wippersnapper; ///< Forward declaration
class GPSModel;      ///< Forward declaration
class GPSHardware;   ///< Forward declaration
class UARTHardware;  ///< Forward declaration

/*!
    @brief  Routes messages between the GPS.proto API and the hardware.
*/
class GPSController {
public:
  GPSController();
  ~GPSController();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_GpsDeviceAddOrReplace(ws_gps_DeviceAddOrReplace *msg);
  bool Handle_GpsDeviceRemove(ws_gps_DeviceRemove *msg);
  // GPS transport setup
  bool AddGPS(HardwareSerial *serial, ws_gps_Config *gps_config);
  bool AddGPS(TwoWire *wire, uint32_t i2c_addr, ws_gps_Config *gps_config);
  // Polling
  void update(bool force = false);
  bool UpdateComplete();
  void ResetFlags();

private:
  GPSModel *_gps_model;                    ///< GPS model instance
  std::vector<GPSHardware *> _gps_drivers; ///< GPS hardware instances
  std::vector<UARTHardware *> _uart_ports; ///< UART hardware for GPS serial
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_GPS_CONTROLLER_H