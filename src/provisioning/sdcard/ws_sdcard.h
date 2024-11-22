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
#include "Wippersnapper_V2.h"

#define SD_FAT_TYPE 3

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
  bool InitSDCard();
  bool ConfigureRTC(const char *rtc_type);
  bool parseConfigFile();
  bool waitForSerialConfig();
  bool validateJson(const char *input);
  bool mode_offline; // TODO: Refactor to getter/setter
  uint32_t GetTimestamp();
  // Encoders for SD card logging
  // GPIO (Analog and Digital Pin) Events
  bool LogGPIOSensorEventToSD(uint8_t pin, float value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, bool value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, uint16_t value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogDS18xSensorEventToSD(wippersnapper_ds18x20_Ds18x20Event *event_msg);

private:
  bool InitDS1307();
  bool InitDS3231();
  bool InitPCF8523();
  bool InitSoftRTC();
  void CheckIn(uint8_t max_digital_pins, uint8_t max_analog_pins,
               float ref_voltage);
  SdFat _sd;                  ///< SD object from Adafruit SDFat library
  String _serialInput;        ///< Serial input buffer
  const char *json_test_data; ///< Json test data
  bool _use_test_data; ///< True if sample data is being used to test, instead
                       ///< of serial input, False otherwise.
  bool _wokwi_runner;  ///< True if `exportedBy` key is "wokwi", otherwise False
  RTC_DS3231 *_rtc_ds3231 = nullptr;   ///< DS3231 RTC object
  RTC_DS1307 *_rtc_ds1307 = nullptr;   ///< DS1307 RTC object
  RTC_PCF8523 *_rtc_pcf8523 = nullptr; ///< PCF8523 RTC object
  RTC_Millis *_rtc_soft = nullptr;     ///< Software RTC object
};
extern Wippersnapper_V2 WsV2;
#endif // WS_SDCARD_H