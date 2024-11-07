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
  void EnableLogging();
  bool parseConfigFile();
  bool waitForSerialConfig();
  bool validateJson(const char *input);
  bool mode_offline;
  // Encoders for SD card logging
  // GPIO (Analog and Digital Pin) Events
  bool LogGPIOSensorEventToSD(uint8_t pin, float value,
                              wippersnapper_sensor_SensorType read_type);
  bool LogGPIOSensorEventToSD(uint8_t pin, bool value,
                              wippersnapper_sensor_SensorType read_type);
  // Logging
  // TODO:
  // 1) Create a logging file on the SD
  // 2) Create a func to decode pb data to json string
  // 3) Log the json string x`xto the file
private:
  SdFat _sd;                  ///< SD object from Adafruit SDFat library
  String _serialInput;        ///< Serial input buffer
  const char *json_test_data; ///< Json test data
  bool _use_test_data; ///< True if sample data is being used to test, instead
                       ///< of serial input, False otherwise.
  RTC_DS3231 *_rtc_ds3231 = nullptr;   ///< DS3231 RTC object
  RTC_DS1307 *_rtc_ds1307 = nullptr;   ///< DS1307 RTC object
  RTC_PCF8523 *_rtc_pcf8523 = nullptr; ///< PCF8523 RTC object
};
extern Wippersnapper_V2 WsV2;
#endif // WS_SDCARD_H