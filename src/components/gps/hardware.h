/*!
 * @file src/components/gps/hardware.h
 *
 * Low-level hardware interface for WipperSnapper's generic GPS component.
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
#ifndef WS_GPS_HARDWARE_H
#define WS_GPS_HARDWARE_H
#include "Wippersnapper_V2.h"
#include <Adafruit_GPS.h>

#define CMD_MTK_QUERY_FW                                                       \
  "$PMTK605*31" ///< Request to query MediaTek firmware version
#define CMD_MTK_QUERY_FW_RESP                                                  \
  "$PMTK705" ///< Response from querying MediaTek firmware version without the
             ///< ReleaseStr
#define MAX_NEMA_SENTENCE_LEN 82 ///< Maximum length of a NMEA sentence

class Wippersnapper_V2; ///< Forward declaration
class UARTHardware;     ///< Forward declaration

enum GpsInterfaceType {
  GPS_IFACE_NONE,    ///< No interface/undefined
  GPS_IFACE_UART_HW, ///< UART hardware interface
  GPS_IFACE_UART_SW, ///< UART software interface
  GPS_IFACE_I2C      ///< I2C interface
}; ///< Type of interface used by GPS

enum GpsDriverType {
  GPS_DRV_NONE,        ///< No driver/undefined
  GPS_DRV_MTK,         ///< MediaTek GPS driver
  GPS_DRV_UBLOX,       ///< u-blox GPS driver
  GPS_DRV_GENERIC_NMEA ///< Generic NMEA GPS driver
}; ///< Type of GPS driver used

/*!
    @brief  Low-level hardware interface for WipperSnapper's generic GPS
            component. This class handles the communication with the GPS module
            over a specified interface (UART, I2C, etc.) and manages the GPS
            driver type.
*/
class GPSHardware {
public:
  GPSHardware();
  ~GPSHardware();
  bool begin();
  bool SetInterface(HardwareSerial *serial);
  // TODO: Add SetInterface(I2C *_i2c_hardware) for I2C support here!
  bool Handle_GPSConfig(wippersnapper_gps_GPSConfig *gps_config);

private:
  bool QueryModuleType();
  bool DetectMediatek();
  bool BuildPmtkAck(char *msg_cmd, char *msg_resp);
  GpsInterfaceType _iface_type;         ///< Type of interface used by GPS
  GpsDriverType _driver_type;           ///< Type of GPS driver used
  HardwareSerial *_hw_serial = nullptr; ///< HardwareSerial instance for GPS;
  Adafruit_GPS *_ada_gps = nullptr;     ///< Adafruit GPS instance
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_GPS_HARDWARE_H