/*!
 * @file ws_sdcard.h
 *
 * Interface for Wippersnapper's SD card filesystem.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SDCARD_H
#define WS_SDCARD_H
#include "Wippersnapper_V2.h"
#include "RTClib.h"
#include "SdFat.h"
#include "StreamUtils.h"
#include "sdios.h"

#if defined(ARDUINO_FEATHER_ESP32) ||                                          \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) ||                               \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
#define SPI_SD_CLOCK                                                           \
  SD_SCK_MHZ(25) ///< For ESP32/Pico silicon rev 3.0, we clock at 25MHz
#else
#define SPI_SD_CLOCK SD_SCK_MHZ(50) ///< Default SPI clock speed
#endif

#define SD_FAT_TYPE 3           ///< SD type (3 = FAT16/FAT32 and exFAT)
#define PIN_SD_CS_ERROR 255     ///< Error code for invalid SD card CS pin
#define UNKNOWN_VALUE "unknown" ///< Default unknown JSON field value
#define MAX_SZ_LOG_FILE (512 * 1024 * 1024) ///< Maximum log file size, in Bytes
#define MAX_LEN_CFG_JSON                                                       \
  4096 ///< Maximum length of the configuration JSON file, in Bytes

// forward decl.
class Wippersnapper_V2;

/***************************************************************************/
/*!
    @brief  Class that handles Wippersnapper's optional filesystem commands
                and storage.
*/
/***************************************************************************/
class ws_sdcard {
public:
  ws_sdcard();
  ~ws_sdcard();
  bool isSDCardInitialized() { return is_mode_offline; }
  bool parseConfigFile();
  bool CreateNewLogFile();
  bool isModeOffline() { return is_mode_offline; }
  void waitForSerialConfig();
  bool LogGPIOSensorEventToSD(uint8_t pin, float value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, bool value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, uint16_t value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogDS18xSensorEventToSD(wippersnapper_ds18x20_Ds18x20Event *event_msg);
private:
  void calculateFileLimits();
  bool ValidateChecksum(JsonDocument &doc);
  bool ValidateJSONKey(const char *key, const char *error_msg);
  void CheckIn(uint8_t max_digital_pins, uint8_t max_analog_pins,
               float ref_voltage);
  bool ConfigureRTC(const char *rtc_type);
  uint32_t GetTimestamp();
  bool InitDS1307();
  bool InitDS3231();
  bool InitPCF8523();
  bool InitSoftRTC();
  wippersnapper_sensor_SensorType ParseSensorType(const char *sensor_type);
  bool ParseDigitalIOAdd(wippersnapper_digitalio_DigitalIOAdd &msg_DigitalIOAdd,
                         const char *pin, float period, bool value,
                         const char *sample_mode, const char *direction,
                         const char *pull);
  bool ParseAnalogIOAdd(wippersnapper_analogio_AnalogIOAdd &msg_AnalogIOAdd,
                        const char *pin, float period, const char *mode);
  bool ParseDS18X20Add(wippersnapper_ds18x20_Ds18x20Add &msg_DS18X20Add,
                       const char *pin, int resolution, float period,
                       int num_sensors, const char *sensor_type_1,
                       const char *sensor_type_2);
  bool ParseI2cDeviceAddReplace(JsonObject &component, wippersnapper_i2c_I2cDeviceAddOrReplace &msg_i2c_device_add_replace);
  uint32_t HexStrToInt(const char *hex_str);

  void BuildJSONDoc(JsonDocument &doc, uint8_t pin, float value,
                    wippersnapper_sensor_SensorType read_type);
  void BuildJSONDoc(JsonDocument &doc, uint8_t pin, uint16_t value,
                    wippersnapper_sensor_SensorType read_type);
  void BuildJSONDoc(JsonDocument &doc, uint8_t pin, bool value,
                    wippersnapper_sensor_SensorType read_type);
  bool LogJSONDoc(JsonDocument &doc);
  bool AddSignalMessageToSharedBuffer(
      wippersnapper_signal_BrokerToDevice &msg_signal);

  SdSpiConfig _sd_spi_cfg; ///< SPI configuration for the SD card
  SdFat _sd;               ///< SD object from Adafruit SDFat library
  size_t _sd_capacity;     ///< Capacity of the SD card, in Bytes
  size_t _sz_cur_log_file; ///< Size of the current log file, in Bytes
  size_t _max_sz_log_file; ///< Calculated maximum size of a log file, in Bytes
  int _sd_max_num_log_files; ///< Maximum number of log files that can fit on
                             ///< the SD card
  int _sd_cur_log_files; ///< Current number of log files that can fit on the SD
                         ///< card
  bool is_mode_offline;  ///< True if offline mode is enabled, False otherwise
  String _serialInput;   ///< Serial input buffer
  const char *json_test_data;          ///< Json test data
  const char *_log_filename;           ///< Path to the log file
  RTC_DS3231 *_rtc_ds3231 = nullptr;   ///< DS3231 RTC object
  RTC_DS1307 *_rtc_ds1307 = nullptr;   ///< DS1307 RTC object
  RTC_PCF8523 *_rtc_pcf8523 = nullptr; ///< PCF8523 RTC object
  RTC_Millis *_rtc_soft = nullptr;     ///< Software RTC object
  bool _use_test_data; ///< True if sample data is being used for testing
};
extern Wippersnapper_V2 WsV2;
#endif // WS_SDCARD_H