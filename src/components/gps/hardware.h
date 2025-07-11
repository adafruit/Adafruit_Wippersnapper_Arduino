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
#include <MicroNMEA.h>
#include <SparkFun_u-blox_GNSS_v3.h>

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
#define UBX_I2C_ADDRESS 0x42     ///< I2C address for all u-Blox GPS products
#define MAX_NMEA_SENTENCES 10    ///< Size of the NMEA buffer
#define MAX_LEN_NMEA_SENTENCE 82 ///< Maximum length of a NMEA sentence

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

typedef struct {
  char sentences[MAX_NMEA_SENTENCES][MAX_LEN_NMEA_SENTENCE];
  int head;
  int tail;
  int maxlen;
} nmea_buffer_t; ///< NMEA sentence ring buffer structure

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
  SFE_UBLOX_GNSS *GetUbxGps();
  GpsDriverType GetDriverType();
  GpsInterfaceType GetIfaceType();
  int NmeaBufPop(char *sentence);
  // "Helpers" for GPS Drivers
  // Used to abstract common parsing functions from GPS driver libraries (i.e:
  // Adafruit_GPS, SFE_UBLOX_GNSS, anything in the future) and intelligently
  // handle the differences between them
  void ReadDiscardBuffer();
  void PollStoreSentences();
  bool ParseNMEASentence(char *sentence);
  // Datetime getters
  uint8_t GetHour();
  uint8_t GetMinute();
  uint8_t GetSeconds();
  uint16_t GetMilliseconds();
  uint8_t GetDay();
  uint8_t GetMonth();
  uint8_t GetYear();
  // RMC/GGA getters
  bool GetFix();
  float GetLat();
  char GetLatDir();
  float GetLon();
  char GetLonDir();
  uint8_t GetNumSats();
  float GetHDOP();
  float GetAltitude();
  float GetSpeed();
  float GetAngle();
  float GetGeoidHeight();

private:
  bool QueryModuleType();
  bool DetectMtkUart();
  bool DetectMtkI2C(uint32_t addr);
  bool DetectUbxI2C(uint32_t addr);
  bool BuildPmtkAck(char *msg_cmd, char *msg_resp);
  void I2cReadDiscard();
  void UartReadDiscard();
  GpsInterfaceType _iface_type;         ///< Type of interface used by GPS
  GpsDriverType _driver_type;           ///< Type of GPS driver used by GPS
  HardwareSerial *_hw_serial = nullptr; ///< Optional HardwareSerial instance
  TwoWire *_wire = nullptr;             ///< Optional TwoWire instance
  Adafruit_GPS *_ada_gps = nullptr;     ///< Optional Adafruit GPS instance
  SFE_UBLOX_GNSS *_sfe_gps; ///< Optional Sparkfun u-blox GPS instance
  uint32_t _addr;           ///< Optional i2c address
  ulong _period;     ///< Polling period for GPS data (Specified by IO), in ms
  ulong _period_prv; ///< Previous period for GPS data (Specified by IO), in ms
  ulong _kat_prv;    ///< Last time the GPS hardware was polled, in ms
  int _nmea_update_rate; ///< NMEA update rate for GPS data, in Hz
  int _nmea_baud_rate;   ///< NMEA baud rate for GPS data, in bits per second
  MicroNMEA
      *_micro_nmea; ///< Optional MicroNMEA instance for parsing NMEA sentences
  char _micro_nmea_buf[100]; ///< Optional Buffer for MicroNMEA parsing
  // NMEA sentence ring buffer
  int NmeaBufPush(
      const char *new_sentence); ///< Push a sentence to the NMEA buffer
  nmea_buffer_t _nmea_buff;      ///< NMEA buffer for storing sentences
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_GPS_HARDWARE_H