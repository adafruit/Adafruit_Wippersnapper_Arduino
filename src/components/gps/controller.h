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

class Wippersnapper_V2; ///< Forward declaration
class GPSModel;         ///< Forward declaration
class UARTHardware;     ///< Forward declaration

enum GpsInterfaceType {
  GPS_IFACE_NONE,    ///< No interface/undefined
  GPS_IFACE_UART_HW, ///< UART hardware interface
  GPS_IFACE_UART_SW, ///< UART software interface
  GPS_IFACE_I2C      ///< I2C interface
};                   ///< Type of interface used by GPS

/*!
    @brief  Routes messages between the GPS.proto API and the hardware.
*/
class GPSController {
public:
  GPSController();
  ~GPSController();
  bool SetInterface(UARTHardware *uart_hardware);
  // TODO: Add I2C interface support via a ctor right here
  bool GPSController::IsAvailable();
  bool Send_Command(const char *command, unsigned long timeout_ms = 1000);
  bool Handle_GPSConfig(pb_istream_t *stream);
  bool RemoveGPSDevice(const char *id);
  void update();

private:
  GPSModel *_gps_model;                   ///< GPS model
  UARTHardware *_uart_hardware = nullptr; ///< UART hardware instance for GPS
  GpsInterfaceType _iface_type;           ///< Type of interface used by GPS
  Adafruit_GPS _ada_gps = nullptr;        ///< Adafruit GPS instance
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_GPS_CONTROLLER_H