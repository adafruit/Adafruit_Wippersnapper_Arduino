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
#include "RTClib.h"
#include "SdFat.h"
#include "StreamUtils.h"
#include "Wippersnapper_V2.h"

#define SD_FAT_TYPE 3           ///< SdFat type (3 = SdFs)
#define MAX_LOG_FILE_SZ 500     ///< Maximum log file size of 500 bytes
#define UNKNOWN_VALUE "unknown" ///< Unknown JSON field value

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
  bool InitSDCard(uint8_t pin_cs);
  bool parseConfigFile();
#ifdef OFFLINE_MODE_DEBUG
  bool waitForSerialConfig();
#endif
  bool CreateNewLogFile();
  bool isModeOffline() { return is_mode_offline; }
  bool LogGPIOSensorEventToSD(uint8_t pin, float value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, bool value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, uint16_t value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogDS18xSensorEventToSD(wippersnapper_ds18x20_Ds18x20Event *event_msg);

private:
  bool ValidateChecksum(JsonDocument &doc);
  bool ValidateJSON(const char *input);
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
  void BuildJSONDoc(JsonDocument &doc, uint8_t pin, float value,
                    wippersnapper_sensor_SensorType read_type);
  void BuildJSONDoc(JsonDocument &doc, uint8_t pin, uint16_t value,
                    wippersnapper_sensor_SensorType read_type);
  void BuildJSONDoc(JsonDocument &doc, uint8_t pin, bool value,
                    wippersnapper_sensor_SensorType read_type);
  bool LogJSONDoc(JsonDocument &doc);
  bool
  PushSignalToSharedBuffer(wippersnapper_signal_BrokerToDevice &msg_signal);
  SdFat _sd;            ///< SD object from Adafruit SDFat library
  bool is_mode_offline; ///< True if offline mode is enabled, False otherwise
  String _serialInput;  ///< Serial input buffer
  const char *json_test_data;        ///< Json test data
  const char *_log_filename;         ///< Path to the log file
  size_t _sz_log_file;               ///< Size of the current log file, in Bytes
  RTC_DS3231 *_rtc_ds3231 = nullptr; ///< DS3231 RTC object
  RTC_DS1307 *_rtc_ds1307 = nullptr; ///< DS1307 RTC object
  RTC_PCF8523 *_rtc_pcf8523 = nullptr; ///< PCF8523 RTC object
  RTC_Millis *_rtc_soft = nullptr;     ///< Software RTC object
  bool _use_test_data; ///< True if sample data is being used for testing
};
extern Wippersnapper_V2 WsV2;
#endif // WS_SDCARD_H