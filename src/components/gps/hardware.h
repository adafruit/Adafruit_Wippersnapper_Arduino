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
#define CMD_MTK_CHECK_ANTENNA "$PGCMD,33,1*6C" ///< Command to check antenna
#define DEFAULT_MTK_NMEA_UPDATE_RATE 1  ///< Default NMEA update rate in Hz
#define DEFAULT_MTK_NMEA_BAUD_RATE 9600 ///< Default NMEA baud rate in bits per
#define MAX_NEMA_SENTENCE_LEN 82        ///< Maximum length of a NMEA sentence
#define PA1010D_I2C_ADDRESS 0x10        ///< I2C address for PA1010D GPS module

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
  bool SetInterface(TwoWire *wire);
  void SetPollPeriod(ulong poll_period);
  void SetPollPeriodPrv(ulong poll_period_prv);
  ulong GetPollPeriod();
  ulong GetPollPeriodPrv();
  ulong GetPrvKat();
  void SetPrvKat(ulong kat_prv);
  void SetNmeaUpdateRate(int nmea_update_rate);
  int GetNmeaUpdateRate();
  void SetNmeaBaudRate(int nmea_baud_rate);
  int GetNmeaBaudRate();
  void SetI2CAddress(uint32_t i2c_address);
  bool Handle_GPSConfig(wippersnapper_gps_GPSConfig *gps_config);
  Adafruit_GPS *GetAdaGps();
  GpsDriverType GetDriverType();

private:
  bool QueryModuleType();
  bool DetectMediatek();
  bool BuildPmtkAck(char *msg_cmd, char *msg_resp);
  GpsInterfaceType _iface_type;         ///< Type of interface used by GPS
  GpsDriverType _driver_type;           ///< Type of GPS driver used
  HardwareSerial *_hw_serial = nullptr; ///< HardwareSerial instance for GPS;
  TwoWire *_wire = nullptr;             ///< TwoWire instance for I2C GPS
  Adafruit_GPS *_ada_gps = nullptr;     ///< Adafruit GPS instance
  uint32_t _addr;                       ///< I2C address for GPS device
  ulong _period;     ///< Polling period for GPS data (Specified by IO), in ms
  ulong _period_prv; ///< Previous period for GPS data (Specified by IO), in ms
  ulong _kat_prv;    ///< Last time the GPS hardware was polled, in ms
  int _nmea_update_rate; ///< NMEA update rate for GPS data, in Hz
  int _nmea_baud_rate;   ///< NMEA baud rate for GPS data, in bits per second
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_GPS_HARDWARE_H