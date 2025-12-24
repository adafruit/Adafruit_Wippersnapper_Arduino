/*!
 * @file ws_sdcard.cpp
 *
 * Interface for Wippersnapper's SD card filesystem.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024-2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "ws_sdcard.h"

/*!
    @brief    Constructs an instance of the Wippersnapper SD card class.
*/
ws_sdcard::ws_sdcard()
#ifdef SD_USE_SPI_1
    : _sd_spi_cfg(WsV2.pin_sd_cs, DEDICATED_SPI, SPI_SD_CLOCK, &SPI1) {
#else
    : _sd_spi_cfg(WsV2.pin_sd_cs, DEDICATED_SPI, SPI_SD_CLOCK) {
#endif
  is_mode_offline = false;
  _use_test_data = false;
  _is_soft_rtc = false;
  _sz_cur_log_file = 0;
  _sd_cur_log_files = 0;

  if (WsV2.pin_sd_cs == PIN_SD_CS_ERROR)
    return;

  if (!_sd.begin(_sd_spi_cfg)) {
    WS_DEBUG_PRINTLN(
        "[SD] Runtime Error: SD initialization failed.\nDo not reformat the "
        "card!\nIs the card "
        "correctly inserted?\nIs there a wiring/soldering problem\n");
    is_mode_offline = false;
    return;
  }

  // Card initialized - calculate file limits
  is_mode_offline = true;
  calculateFileLimits();
}

/*!
    @brief    Destructs an instance of the Wippersnapper SD card class.
*/
ws_sdcard::~ws_sdcard() {
  if (is_mode_offline) {
    _sd.end(); // Close the SD card interface
  }
  is_mode_offline = false;
}

void ws_sdcard::calculateFileLimits() {
  // Calculate the maximum number of log files that can be stored on the SD card
  csd_t csd;
  if (!_sd.card()->readCSD(&csd)) {
    WS_DEBUG_PRINTLN("[SD] Runtime Error: Could not read card information");
    return;
  }

  // get the complete sdcard capacity in bytes
  _sd_capacity = (uint64_t)512 * csd.capacity();
  // account for 3-5% fatfs overhead utilization
  size_t sd_capacity_usable = _sd_capacity * (1 - 0.05);
  // proportionally set sz of each log file to 10% of the SD card's usable
  // capacity
  _max_sz_log_file = sd_capacity_usable / 10;
  // Regardless of sd card size, cap log files to 512MB
  if (_max_sz_log_file > MAX_SZ_LOG_FILE) {
    _max_sz_log_file = MAX_SZ_LOG_FILE;
  }
  _sd_max_num_log_files = sd_capacity_usable / _max_sz_log_file;
}
/*!
    @brief    Initializes a DS1307 RTC
    @returns  True if the RTC was successfully initialized, False otherwise.
*/
bool ws_sdcard::InitDS1307() {
  _rtc_ds1307 = new RTC_DS1307();
  if (!_rtc_ds1307->begin()) {
#if !defined(ARDUINO_ARCH_ESP8266) && !defined(ARDUINO_ARCH_SAMD) &&           \
    !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6) &&                              \
    !defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
    if (!_rtc_ds1307->begin(&Wire1)) {
      WS_DEBUG_PRINTLN("[SD] Runtime Error: Failed to initialize DS1307 RTC");
      delete _rtc_ds1307;
      return false;
    }
#endif
  }
  if (!_rtc_ds1307->isrunning())
    _rtc_ds1307->adjust(DateTime(F(__DATE__), F(__TIME__)));
  return true;
}

/*!
    @brief    Initializes a DS3231 RTC.
    @returns  True if the RTC was successfully initialized, False
              otherwise.
*/
bool ws_sdcard::InitDS3231() {
  WS_DEBUG_PRINTLN("Begin DS3231 init");
  _rtc_ds3231 = new RTC_DS3231();
  if (!_rtc_ds3231->begin(&Wire)) {
#if !defined(ARDUINO_ARCH_ESP8266) && !defined(ARDUINO_ARCH_SAMD) &&           \
    !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6) &&                              \
    !defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
    if (!_rtc_ds3231->begin(&Wire1)) {
      WS_DEBUG_PRINTLN("[SD] Runtime Error: Failed to initialize DS3231 RTC");
      delete _rtc_ds3231;
      return false;
    }
#endif
  }
  if (_rtc_ds3231->lostPower())
    _rtc_ds3231->adjust(DateTime(F(__DATE__), F(__TIME__)));
  return true;
}

/*!
    @brief    Initializes a PCF8523 RTC.
    @returns  True if the RTC was successfully initialized, False
              otherwise.
*/
bool ws_sdcard::InitPCF8523() {
  _rtc_pcf8523 = new RTC_PCF8523();
  if (!_rtc_pcf8523->begin(&Wire)) {
    WS_DEBUG_PRINTLN(
        "[SD] Runtime Error: Failed to initialize PCF8523 RTC on WIRE");
#if !defined(ARDUINO_ARCH_ESP8266) && !defined(ARDUINO_ARCH_SAMD) &&           \
    !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6) &&                              \
    !defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
    if (!_rtc_pcf8523->begin(&Wire1)) {
      WS_DEBUG_PRINTLN(
          "[SD] Runtime Error: Failed to initialize PCF8523 RTC on WIRE1");
      delete _rtc_pcf8523;
      return false;
    }
#endif
  }
  if (!_rtc_pcf8523->initialized() || _rtc_pcf8523->lostPower()) {
    _rtc_pcf8523->adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  _rtc_pcf8523->start();
  return true;
}

/*!
    @brief    Initializes a "soft" RTC for devices without a physical
              RTC module attached.
    @returns  True if the soft RTC was successfully initialized, False
              otherwise.
*/
bool ws_sdcard::InitSoftRTC() {
  _is_soft_rtc = true;
  _soft_rtc_counter = 0;
  return _is_soft_rtc;
}

/*!
    @brief    Increments the "soft" RTC.
*/
void ws_sdcard::TickSoftRTC() { _soft_rtc_counter++; }

/*!
    @brief    Returns the current timestamp from the RTC.
    @returns  The current timestamp from the RTC.
*/
uint32_t ws_sdcard::GetSoftRTCTime() { return _soft_rtc_counter; }

/*!
    @brief  Initializes and configures a RTC for logging.
    @param  rtc_type
            The desired type of RTC to configure.
    @returns True if the RTC was successfully configured, False otherwise.
*/
bool ws_sdcard::ConfigureRTC(const char *rtc_type) {
  if (strcmp(rtc_type, "DS1307") == 0) {
    return InitDS1307();
  } else if (strcmp(rtc_type, "DS3231") == 0) {
    return InitDS3231();
  } else if (strcmp(rtc_type, "PCF8523") == 0) {
    return InitPCF8523();
  } else if (strcmp(rtc_type, "SOFT") == 0) {
    return InitSoftRTC();
  } else
    WS_DEBUG_PRINTLN(
        "[SD] Runtime Error: Unknown RTC type found in JSON string!");
  return false;
}

/*!
    @brief  Mocks checking in with Adafruit IO servers
    @param  exported_from_device
            The JSON object containing the device configuration.
*/
void ws_sdcard::CheckIn(const JsonObject &exported_from_device) {
  // Configure controllers
  WsV2.digital_io_controller->SetMaxDigitalPins(
      exported_from_device["maxDigitalPins"] | 0);
  WsV2.analogio_controller->SetTotalAnalogPins(
      exported_from_device["maxAnalogPins"] | 0);
  WsV2.analogio_controller->SetRefVoltage(exported_from_device["refVoltage"] |
                                          0.0f);
  // Since `secrets.json` is unused in offline mode, use the status LED
  // brightness from here instead
  setStatusLEDBrightness(exported_from_device["statusLEDBrightness"] | 0.3f);
}

/*!
    @brief  Parses a sensor type from the JSON configuration file.
    @param  sensor_type
            The sensor type to parse.
    @returns The corresponding SensorType
*/
ws_sensor_Type ws_sdcard::ParseSensorType(const char *sensor_type) {
  if (strcmp(sensor_type, "raw") == 0) {
    return ws_sensor_Type_T_RAW;
  } else if (strcmp(sensor_type, "voltage") == 0) {
    return ws_sensor_Type_T_VOLTAGE;
  } else if (strcmp(sensor_type, "current") == 0) {
    return ws_sensor_Type_T_CURRENT;
  } else if (strcmp(sensor_type, "object-temp-fahrenheit") == 0) {
    return ws_sensor_Type_T_OBJECT_TEMPERATURE_FAHRENHEIT;
  } else if (strcmp(sensor_type, "object-temp") == 0) {
    return ws_sensor_Type_T_OBJECT_TEMPERATURE;
  } else if (strcmp(sensor_type, "ambient-temp") == 0) {
    return ws_sensor_Type_T_AMBIENT_TEMPERATURE;
  } else if (strcmp(sensor_type, "ambient-temp-fahrenheit") == 0) {
    return ws_sensor_Type_T_AMBIENT_TEMPERATURE_FAHRENHEIT;
  } else if (strcmp(sensor_type, "accelerometer") == 0) {
    return ws_sensor_Type_T_ACCELEROMETER;
  } else if (strcmp(sensor_type, "magnetic-field") == 0) {
    return ws_sensor_Type_T_MAGNETIC_FIELD;
  } else if (strcmp(sensor_type, "orientation") == 0) {
    return ws_sensor_Type_T_ORIENTATION;
  } else if (strcmp(sensor_type, "gyroscope") == 0) {
    return ws_sensor_Type_T_GYROSCOPE;
  } else if (strcmp(sensor_type, "gravity") == 0) {
    return ws_sensor_Type_T_GRAVITY;
  } else if (strcmp(sensor_type, "linear-acceleration") == 0) {
    return ws_sensor_Type_T_LINEAR_ACCELERATION;
  } else if (strcmp(sensor_type, "rotation-vector") == 0) {
    return ws_sensor_Type_T_ROTATION_VECTOR;
  } else if (strcmp(sensor_type, "altitude") == 0) {
    return ws_sensor_Type_T_ALTITUDE;
  } else if (strcmp(sensor_type, "relative-humidity") == 0) {
    return ws_sensor_Type_T_RELATIVE_HUMIDITY;
  } else if (strcmp(sensor_type, "pressure") == 0) {
    return ws_sensor_Type_T_PRESSURE;
  } else if (strcmp(sensor_type, "light") == 0) {
    return ws_sensor_Type_T_LIGHT;
  } else if (strcmp(sensor_type, "lux") == 0) {
    return ws_sensor_Type_T_LUX;
  } else if (strcmp(sensor_type, "proximity") == 0) {
    return ws_sensor_Type_T_PROXIMITY;
  } else if (strcmp(sensor_type, "pm10-std") == 0) {
    return ws_sensor_Type_T_PM10_STD;
  } else if (strcmp(sensor_type, "pm25-std") == 0) {
    return ws_sensor_Type_T_PM25_STD;
  } else if (strcmp(sensor_type, "pm100-std") == 0) {
    return ws_sensor_Type_T_PM100_STD;
  } else if (strcmp(sensor_type, "pm10-env") == 0) {
    return ws_sensor_Type_T_PM10_ENV;
  } else if (strcmp(sensor_type, "pm25-env") == 0) {
    return ws_sensor_Type_T_PM25_ENV;
  } else if (strcmp(sensor_type, "pm100-env") == 0) {
    return ws_sensor_Type_T_PM100_ENV;
  } else if (strcmp(sensor_type, "co2") == 0) {
    return ws_sensor_Type_T_CO2;
  } else if (strcmp(sensor_type, "eco2") == 0) {
    return ws_sensor_Type_T_ECO2;
  } else if (strcmp(sensor_type, "gas-resistance") == 0) {
    return ws_sensor_Type_T_GAS_RESISTANCE;
  } else if (strcmp(sensor_type, "voc-index") == 0) {
    return ws_sensor_Type_T_VOC_INDEX;
  } else if (strcmp(sensor_type, "nox-index") == 0) {
    return ws_sensor_Type_T_NOX_INDEX;
  } else if (strcmp(sensor_type, "tvoc") == 0) {
    return ws_sensor_Type_T_TVOC;
  } else if (strcmp(sensor_type, "color") == 0) {
    return ws_sensor_Type_T_COLOR;
  } else if (strcmp(sensor_type, "unitless-percent") == 0) {
    return ws_sensor_Type_T_UNITLESS_PERCENT;
  } else if (strcmp(sensor_type, "bytes") == 0) {
    return ws_sensor_Type_T_BYTES;
  } else if (strcmp(sensor_type, "boolean") == 0) {
    return ws_sensor_Type_T_BOOLEAN;
  } else {
    WS_DEBUG_PRINT("[SD] ERROR: Found unspecified SensorType - ");
    WS_DEBUG_PRINTLN(sensor_type);
    return ws_sensor_Type_T_UNSPECIFIED;
  }
}

bool ws_sdcard::ValidateJSONKey(const char *key, const char *error_msg) {
  if (strcmp(key, UNKNOWN_VALUE) == 0) {
    WS_DEBUG_PRINTLN(error_msg);
    return false;
  }
  return true;
}

/*!
    @brief  Parses a DigitalIOAdd message from the JSON configuration file.
    @param  msg_DigitalIOAdd
            The DigitalIOAdd message to populate.
    @param  pin
            The GPIO pin name.
    @param  period
            The desired period to read the sensor, in seconds.
    @param  value
            The sensor value.
    @param  sample_mode
            The sample mode.
    @param  direction
            The GPIO pin direction.
    @param  pull
            The GPIO pin pull.
    @returns True if the DigitalIOAdd message was successfully parsed,
             False otherwise.
*/
bool ws_sdcard::ParseDigitalIOAdd(ws_digitalio_Add &msg_DigitalIOAdd,
                                  const char *pin, float period, bool value,
                                  const char *sample_mode,
                                  const char *direction, const char *pull) {
  if (!ValidateJSONKey(pin, "[SD] Parsing Error: Digital pin name not found!"))
    return false;
  strcpy(msg_DigitalIOAdd.pin_name, pin);

  if (period == 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Invalid pin period!");
    return false;
  }
  msg_DigitalIOAdd.period = period;
  msg_DigitalIOAdd.value = value;

  // Determine the sample mode
  if (!ValidateJSONKey(
          sample_mode,
          "[SD] Parsing Error: Digital pin's sample mode not found!")) {
    return false;
  } else if (strcmp(sample_mode, "TIMER") == 0) {
    msg_DigitalIOAdd.sample_mode = ws_digitalio_SampleMode_SM_TIMER;
  } else if (strcmp(sample_mode, "EVENT") == 0) {
    msg_DigitalIOAdd.sample_mode = ws_digitalio_SampleMode_SM_EVENT;
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown sample mode found: " +
                     String(sample_mode));
  }

  // Determine the pin direction and pull
  if (!ValidateJSONKey(
          direction,
          "[SD] Parsing Error: Digital pin's direction not found!")) {
    return false;
  } else if (strcmp(direction, "INPUT") == 0) {
    if (pull != nullptr) {
      msg_DigitalIOAdd.gpio_direction = ws_digitalio_Direction_D_INPUT_PULL_UP;
    } else {
      msg_DigitalIOAdd.gpio_direction = ws_digitalio_Direction_D_INPUT;
    }
  } else if (strcmp(direction, "OUTPUT") == 0) {
    WS_DEBUG_PRINTLN(
        "[SD] Error - Can not set OUTPUT direction in offline mode!");
    return false;
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown direction found: " +
                     String(direction));
    return false;
  }
  return true;
}

/*!
    @brief  Parses an AnalogIOAdd message from the JSON configuration file.
    @param  msg_AnalogIOAdd
            The AnalogIOAdd message to populate.
    @param  pin
            The GPIO pin name.
    @param  period
            The desired period to read the sensor, in seconds.
    @param  mode
            The sensor read mode.
    @returns True if the AnalogIOAdd message was successfully parsed,
             False otherwise.
*/
bool ws_sdcard::ParseAnalogIOAdd(ws_analogio_Add &msg_AnalogIOAdd,
                                 const char *pin, float period,
                                 const char *mode) {

  if (!ValidateJSONKey(pin, "[SD] Parsing Error: Analog pin name not found!"))
    return false;
  strcpy(msg_AnalogIOAdd.pin_name, pin);

  if (period == 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Analog pin period less than 1.0 "
                     "seconds or not found!");
    return false;
  }
  msg_AnalogIOAdd.period = period;

  if (!ValidateJSONKey(mode,
                       "[SD] Parsing Error: Analog pin read mode not found!"))
    return false;
  msg_AnalogIOAdd.read_mode = ParseSensorType(mode);
  if (msg_AnalogIOAdd.read_mode == ws_sensor_Type_T_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown read mode found: " +
                     String(mode));
    return false;
  }
  return true;
}

bool ws_sdcard::ParseDS18X20Add(ws_ds18x20_Add &msg_DS18X20Add, const char *pin,
                                int resolution, float period, int num_sensors,
                                const char *sensor_type_1,
                                const char *sensor_type_2) {

  if (strcmp(pin, UNKNOWN_VALUE) == 0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: DS18X20 pin name not found!");
    return false;
  }
  strcpy(msg_DS18X20Add.onewire_pin, pin);

  if (resolution == 0) {
    WS_DEBUG_PRINTLN(
        "[SD] Parsing Error: DS18X20 sensor resolution not found!");
    return false;
  }
  msg_DS18X20Add.sensor_resolution = resolution;

  if (period == 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: DS18X20 sensor period not found!");
    return false;
  }
  msg_DS18X20Add.period = period;

  if (num_sensors == 0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: DS18X20 sensor count not found!");
    return false;
  }
  msg_DS18X20Add.sensor_types_count = num_sensors;

  // Parse the first sensor type
  if (strcmp(sensor_type_1, UNKNOWN_VALUE) == 0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: DS18X20 sensor type 1 not found!");
    return false;
  }
  msg_DS18X20Add.sensor_types[0] = ParseSensorType(sensor_type_1);
  // Parse the second sensor type, if it exists
  if (num_sensors == 2) {
    if (strcmp(sensor_type_2, UNKNOWN_VALUE) == 0) {
      WS_DEBUG_PRINTLN("[SD] Parsing Error: DS18X20 sensor type 2 not found!");
      return false;
    }
    msg_DS18X20Add.sensor_types[1] = ParseSensorType(sensor_type_2);
  }
  return true;
}

/*!
    @brief  Converts a string-encoded hex value to an integer.
    @param  hex_str
            The string-encoded hex value to convert.
    @returns The integer value of the hex string.
*/
uint32_t ws_sdcard::HexStrToInt(const char *hex_str) {
  return std::stoi(hex_str, nullptr, 16);
}

/*!
    @brief  Parses an I2cDeviceAddOrReplace message from the JSON
            configuration file.
    @param  component
            The JSON object to parse.
    @param  msg_i2c_add
            The I2cDeviceAddOrReplace message to populate.
    @returns True if the I2cDeviceAddOrReplace message was successfully
             parsed, False otherwise.
*/
bool ws_sdcard::ParseI2cDeviceAddReplace(
    JsonObject &component, ws_i2c_DeviceAddOrReplace &msg_i2c_add) {
  strcpy(msg_i2c_add.device_name, component["i2cDeviceName"] | UNKNOWN_VALUE);
  msg_i2c_add.device_period = component["period"] | 0.0;
  if (msg_i2c_add.device_period == 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Invalid I2C device period!");
    return false;
  }

  msg_i2c_add.has_device_description = true;
  strcpy(msg_i2c_add.device_description.bus_scl,
         component["i2cBusScl"] | "default");
  strcpy(msg_i2c_add.device_description.bus_sda,
         component["i2cBusSda"] | "default");

  const char *addr_device = component["i2cDeviceAddress"] | "0x00";
  msg_i2c_add.device_description.device_address = HexStrToInt(addr_device);

  const char *addr_mux = component["i2cMuxAddress"] | "0x00";
  msg_i2c_add.device_description.mux_address = HexStrToInt(addr_mux);

  const char *mux_channel = component["i2cMuxChannel"] | "0xFFFF";
  msg_i2c_add.device_description.mux_channel = HexStrToInt(mux_channel);

  msg_i2c_add.device_sensor_types_count = 0;
  for (JsonObject components_0_i2cDeviceSensorType :
       component["i2cDeviceSensorTypes"].as<JsonArray>()) {
    msg_i2c_add.device_sensor_types[msg_i2c_add.device_sensor_types_count] =
        ParseSensorType(components_0_i2cDeviceSensorType["type"]);
    msg_i2c_add.device_sensor_types_count++;
  }

  return true;
}

/*!
    @brief  Pushes a signal message to the shared buffer.
    @param  msg_signal
            The signal message to push.
    @returns True if the signal message was successfully pushed to the shared
             buffer, False otherwise.
*/
bool ws_sdcard::AddSignalMessageToSharedBuffer(
    ws_signal_BrokerToDevice &msg_signal) {
  // Create a temporary buffer to hold the encoded signal message
  std::vector<uint8_t> tempBuf(512);
  size_t tempBufSz;

  // Get the encoded size of the signal message first so we can resize the
  // buffer prior to encoding
  if (!pb_get_encoded_size(&tempBufSz, ws_signal_BrokerToDevice_fields,
                           &msg_signal)) {
    WS_DEBUG_PRINTLN("[SD] Runtime Error: Unable to get signal message size!");
    return false;
  }

  // Encode and push the signal message to the shared config buffer
  tempBuf.resize(tempBufSz);
  pb_ostream_t ostream = pb_ostream_from_buffer(tempBuf.data(), tempBuf.size());
  if (!ws_pb_encode(&ostream, ws_signal_BrokerToDevice_fields, &msg_signal)) {
    WS_DEBUG_PRINTLN(
        "[SD] Runtime Error: Unable to encode D2B signal message!");
    return false;
  }
  WsV2._sharedConfigBuffers.push_back(std::move(tempBuf));
  return true;
}

/*!
    @brief  Creates a new logging file on the SD card using the RTC's
            timestamp and sets the current log file path to reflect this
            file.
    @returns True if a log file was successfully created, False otherwise.
*/
bool ws_sdcard::CreateNewLogFile() {
  if (_sd_cur_log_files >= _sd_max_num_log_files) {
    WS_DEBUG_PRINTLN("[SD] Runtime Error: Maximum number of log files for SD "
                     "card capacity reached!");
    return false;
  }

  String logFilename;
  // Generate a name for the new log file using the RTC's timestamp
  if (!_is_soft_rtc)
    logFilename = "log_" + String(GetTimestamp()) + ".log";
  else
    logFilename = "log_" + String(millis()) + ".log";
  static char log_filename_buffer[256];
  strncpy(log_filename_buffer, logFilename.c_str(),
          sizeof(log_filename_buffer) - 1);
  log_filename_buffer[sizeof(log_filename_buffer) - 1] = '\0';
  _log_filename = log_filename_buffer;
  _sz_cur_log_file = 0; // Reset the current log file size

  // Attempt to create a new log file
  File32 file;
  if (!file.open(_log_filename, O_RDWR | O_CREAT | O_AT_END))
    return false;
  WS_DEBUG_PRINT("[SD] Created new log file on SD card: ");
  WS_DEBUG_PRINTLN(_log_filename);
  _sd_cur_log_files++;
  return true;
}

bool ws_sdcard::ValidateChecksum(JsonDocument &doc) {
  int json_doc_checksum = doc["checksum"];
  // Calculate the checksum of the JSON document without the checksum field
  doc.remove("checksum");
  String doc_without_checksum;
  serializeJson(doc, doc_without_checksum);
  uint8_t calculated_checksum = 0;
  for (unsigned int i = 0; i < doc_without_checksum.length(); i++) {
    calculated_checksum += doc_without_checksum[i];
  }
  calculated_checksum &= 0xFF; // take lsb

  if (json_doc_checksum != calculated_checksum)
    return false;
  return true;
}

/*!
    @brief  Converts sleep mode and wakeup source strings to their corresponding
   enum values.
    @param  mode_str
            The sleep mode string
    @param  source_str
            The wakeup source string
    @param  mode
            The converted sleep mode enum.
    @param  source
            The converted wakeup source enum.
    @returns True if both conversions were successful, False if either string
   was invalid.
*/
bool ws_sdcard::ConvertSleepStrings(const char *mode_str,
                                    const char *source_str,
                                    ws_sleep_SleepMode &mode,
                                    ws_sleep_WakeupSource &source) {
  // Convert SleepMode to enum
  mode = ws_sleep_SleepMode_S_UNSPECIFIED;
  if (mode_str) {
    if (strcmp(mode_str, "light") == 0) {
      mode = ws_sleep_SleepMode_S_LIGHT;
    } else if (strcmp(mode_str, "deep") == 0) {
      mode = ws_sleep_SleepMode_S_DEEP;
    } else {
      WS_DEBUG_PRINT("[SD] Error: Invalid sleep mode: ");
      WS_DEBUG_PRINTLN(mode_str);
      return false;
    }
  }

  // Convert WakeupSource to enum
  source = ws_sleep_WakeupSource_W_UNSPECIFIED;
  if (source_str) {
    if (strcmp(source_str, "timer") == 0) {
      source = ws_sleep_WakeupSource_W_TIMER;
    } else if (strcmp(source_str, "pin") == 0) {
      source = ws_sleep_WakeupSource_W_PIN;
    } else {
      WS_DEBUG_PRINT("[SD] Error: Invalid wakeup source: ");
      WS_DEBUG_PRINTLN(source_str);
      return false;
    }
  }

  return true;
}

/*!
    @brief  Parses the sleep configuration from the JSON object.
    @param  sleep_config
            The JSON object containing the sleep configuration.
    @param  timer_config
            The JSON object containing the wake timer configuration.
    @param  run_duration
            The duration for which the device should run before sleeping.
    @returns True if the sleep configuration was successfully parsed and
   component configured, False otherwise.
*/
bool ws_sdcard::ParseSleepConfigTimer(const JsonObject &sleep_config,
                                      const JsonObject &timer_config,
                                      int run_duration) {
  bool lock = sleep_config["lock"];
  int duration = timer_config["duration"];

  // Convert strings to enums
  ws_sleep_SleepMode sleep_mode;
  ws_sleep_WakeupSource wakeup_source;
  if (!ConvertSleepStrings(sleep_config["mode"], sleep_config["wakeup"],
                           sleep_mode, wakeup_source)) {
    return false;
  }

  return WsV2._sleep_controller->Configure(lock, sleep_mode, wakeup_source,
                                           duration, run_duration);
}

/*!
    @brief  Parses the sleep configuration from the JSON object.
    @param  sleep_config
            The JSON object containing the sleep configuration.
    @param  pin_config
            The JSON object containing the wake pin configuration.
    @param  run_duration
            The duration for which the device should run before sleeping.
    @returns True if the sleep configuration was successfully parsed and
   component configured, False otherwise.
*/
bool ws_sdcard::ParseSleepConfigPin(const JsonObject &sleep_config,
                                    const JsonObject &pin_config,
                                    int run_duration) {
  bool lock = sleep_config["lock"];
  const char *pin_name = pin_config["name"];
  bool pin_level = pin_config["level"];
  bool pin_pull = pin_config["pull"];

  // Convert strings to enums
  ws_sleep_SleepMode sleep_mode;
  ws_sleep_WakeupSource wakeup_source;
  if (!ConvertSleepStrings(sleep_config["mode"], sleep_config["wakeup"],
                           sleep_mode, wakeup_source)) {
    return false;
  }

  return WsV2._sleep_controller->Configure(lock, sleep_mode, wakeup_source,
                                           pin_name, pin_level, pin_pull,
                                           run_duration);
}

/*!
    @brief  Searches for and parses the JSON configuration file and sets up
            the hardware accordingly.
    @returns True if the JSON file was successfully parsed and the hardware
             was successfully configured. False otherwise.
*/
bool ws_sdcard::parseConfigFile() {
  DeserializationError error;
  JsonDocument doc;
  // delay(5000); // ENABLE FOR TROUBLESHOOTING THIS CLASS ON HARDWARE ONLY

  // Parse configuration data
#ifndef OFFLINE_MODE_DEBUG
  WS_DEBUG_PRINTLN("[SD] Parsing config.json...");
  doc = WsV2._config_doc;
#else
  // Use test data rather than data from the filesystem
  if (!_use_test_data) {
    WS_DEBUG_PRINTLN("[SD] Parsing Serial Input...");
    WS_DEBUG_PRINTLN(_serialInput);
    error = deserializeJson(doc, _serialInput.c_str(), MAX_LEN_CFG_JSON);
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Test Data...");
    error = deserializeJson(doc, json_test_data, MAX_LEN_CFG_JSON);
  }
#endif
  // It is not possible to continue running in offline mode without a valid
  // config file
  if (error) {
    WS_DEBUG_PRINT("[SD] Runtime Error: Unable to deserialize config.json");
    WS_DEBUG_PRINTLN("\nError Code: " + String(error.c_str()));
    return false;
  }
  WS_DEBUG_PRINTLN("[SD] Successfully deserialized JSON config file!");

  // Check config.json file's integrity
  if (!ValidateChecksum(doc)) {
    WS_DEBUG_PRINTLN("[SD] Checksum mismatch, file has been modified from its "
                     "original state!");
    return false;
  }

  // Begin parsing the JSON document
  JsonObject exportedFromDevice = doc["exportedFromDevice"];
  if (exportedFromDevice.isNull()) {
    WS_DEBUG_PRINTLN("[SD] Runtime Error: Required exportedFromDevice not "
                     "found in config file!");
    return false;
  }

  // We don't talk to IO in offline mode, so, mock the device check-in
  CheckIn(exportedFromDevice);

  // Parse sleep configuration, if present
  JsonObject sleep_config = doc["sleepConfig"][0];
  if (!sleep_config.isNull()) {
    bool parse_result = false;
    if (!sleep_config["pinConfig"].isNull()) {
      parse_result = ParseSleepConfigPin(
          sleep_config, sleep_config["pinConfig"], sleep_config["runDuration"]);
    } else if (!sleep_config["timerConfig"].isNull()) {
      parse_result =
          ParseSleepConfigTimer(sleep_config, sleep_config["timerConfig"],
                                sleep_config["runDuration"]);
    } else {
      WS_DEBUG_PRINTLN("[SD] Runtime Error: Missing sleep configuration type!");
      return false;
    }

    if (!parse_result) {
      WS_DEBUG_PRINT("[SD] Failed to parse sleep configuration!");
      return false;
    }
  }

#ifndef OFFLINE_MODE_WOKWI
  const char *json_rtc = exportedFromDevice["rtc"] | "SOFT";
  if (!ConfigureRTC(json_rtc)) {
    WS_DEBUG_PRINTLN("[SD] Runtime Error: Failed to to configure RTC!");
    return false;
  }
#endif

  // Parse each component from JSON->PB and push into a shared buffer
  for (JsonObject component : doc["components"].as<JsonArray>()) {
    ws_signal_BrokerToDevice msg_signal_b2d =
        ws_signal_BrokerToDevice_init_default;

    // Parse the component API type
    const char *component_api_type = component["componentAPI"];
    if (component_api_type == nullptr) {
      WS_DEBUG_PRINTLN("[SD] Runtime Error: Missing component API type!");
      return false;
    }

    // Determine the component type and parse it into a PB message
    if (strcmp(component_api_type, "digitalio") == 0) {
      WS_DEBUG_PRINTLN(
          "[SD] DigitalIO component found, decoding JSON to PB...");
      ws_digitalio_Add msg_DigitalIOAdd = ws_digitalio_Add_init_default;
      if (!ParseDigitalIOAdd(
              msg_DigitalIOAdd, component["pinName"] | UNKNOWN_VALUE,
              component["period"] | 0.0, component["value"],
              component["sampleMode"] | UNKNOWN_VALUE,
              component["direction"] | UNKNOWN_VALUE, component["pull"])) {
        WS_DEBUG_PRINT(
            "[SD] Runtime Error: Unable to parse DigitalIO Component, Pin: ");
        WS_DEBUG_PRINTLN(component["pinName"] | UNKNOWN_VALUE);
        return false;
      }
      // Configure the message envelope
      msg_signal_b2d.which_payload = ws_signal_BrokerToDevice_digitalio_tag;
      msg_signal_b2d.payload.digitalio.payload.add = msg_DigitalIOAdd;
      msg_signal_b2d.payload.digitalio.which_payload = ws_digitalio_B2D_add_tag;
    } else if (strcmp(component_api_type, "analogio") == 0) {
      WS_DEBUG_PRINTLN("[SD] AnalogIO component found, decoding JSON to PB...");
      ws_analogio_Add msg_AnalogIOAdd = ws_analogio_Add_init_default;
      if (!ParseAnalogIOAdd(msg_AnalogIOAdd,
                            component["pinName"] | UNKNOWN_VALUE,
                            component["period"] | 0.0,
                            component["analogReadMode"] | UNKNOWN_VALUE)) {
        WS_DEBUG_PRINTLN(
            "[SD] Runtime Error: Unable to parse AnalogIO Component, Pin: ");
        WS_DEBUG_PRINTLN(component["pinName"] | UNKNOWN_VALUE);
        return false;
      }

      msg_signal_b2d.which_payload = ws_signal_BrokerToDevice_analogio_tag;
      msg_signal_b2d.payload.analogio.payload.add = msg_AnalogIOAdd;
      msg_signal_b2d.payload.analogio.which_payload = ws_analogio_B2D_add_tag;
    } else if (strcmp(component_api_type, "ds18x20") == 0) {
      WS_DEBUG_PRINTLN("[SD] Ds18x20 component found, decoding JSON to PB...");
      ws_ds18x20_Add msg_DS18X20Add = ws_ds18x20_Add_init_default;
      if (!ParseDS18X20Add(msg_DS18X20Add, component["pinName"] | UNKNOWN_VALUE,
                           component["sensorResolution"] | 0,
                           component["period"] | 0.0,
                           component["sensorTypeCount"] | 0,
                           component["sensorType1"] | UNKNOWN_VALUE,
                           component["sensorType2"] | UNKNOWN_VALUE)) {
        WS_DEBUG_PRINTLN(
            "[SD] Runtime Error: Unable to parse DS18X20 Component, Pin: ");
        WS_DEBUG_PRINTLN(component["pinName"] | UNKNOWN_VALUE);
        return false;
      }
      msg_signal_b2d.which_payload = ws_signal_BrokerToDevice_ds18x20_tag;
      msg_signal_b2d.payload.ds18x20.which_payload = ws_ds18x20_B2D_add_tag;
      msg_signal_b2d.payload.ds18x20.payload.add = msg_DS18X20Add;
    } else if (strcmp(component_api_type, "i2c") == 0) {
      WS_DEBUG_PRINTLN("[SD] I2C component found, decoding JSON to PB...");
      ws_i2c_DeviceAddOrReplace msg_i2c_add_replace =
          ws_i2c_DeviceAddOrReplace_init_default;
      if (!ParseI2cDeviceAddReplace(component, msg_i2c_add_replace)) {
        WS_DEBUG_PRINTLN("[SD] Runtime Error: Unable to parse I2C Component");
        return false;
      }
      msg_signal_b2d.which_payload = ws_signal_BrokerToDevice_i2c_tag;
      msg_signal_b2d.payload.i2c.which_payload =
          ws_i2c_B2D_device_add_replace_tag;
      msg_signal_b2d.payload.i2c.payload.device_add_replace =
          msg_i2c_add_replace;
    } else {
      WS_DEBUG_PRINTLN("[SD] Runtime Error: Unknown Component API Type: " +
                       String(component_api_type));
      return false;
    }

    // App handles the signal messages, in-order
    if (!AddSignalMessageToSharedBuffer(msg_signal_b2d)) {
      WS_DEBUG_PRINTLN("[SD] Runtime Error: Unable to add signal message(s) "
                       "to shared buffer!");
      return false;
    }
  }
  return true;
}

/*!
    @brief  Obtains a timestamp from the hardware (or software) RTC.
    @returns The current timestamp, in unixtime format.
*/
uint32_t ws_sdcard::GetTimestamp() {
  DateTime now;
  if (_rtc_ds3231 != nullptr)
    now = _rtc_ds3231->now();
  else if (_rtc_ds1307 != nullptr)
    now = _rtc_ds1307->now();
  else if (_rtc_pcf8523 != nullptr)
    now = _rtc_pcf8523->now();
  else if (_is_soft_rtc) {
    uint32_t cur_time = GetSoftRTCTime();
    TickSoftRTC();
    return cur_time; // early-return as we are not converting this "soft rtc" to
                     // unixtime
  } else { // we're either using a simulator or have undefined behavior
    return 0;
  }

  return now.unixtime();
}

/*!
    @brief  Converts a SensorType enum to a string.
    @param  sensorType
            The SensorType enum to convert.
    @returns A string representation of the SensorType enum.
*/
const char *SensorTypeToSIUnit(ws_sensor_Type sensorType) {
  switch (sensorType) {
  case ws_sensor_Type_T_UNSPECIFIED:
    return "UNSPECIFIED";
  case ws_sensor_Type_T_ACCELEROMETER:
    return "m/s/s";
  case ws_sensor_Type_T_MAGNETIC_FIELD:
    return "µT";
  case ws_sensor_Type_T_ORIENTATION:
    return "\xB0";
  case ws_sensor_Type_T_GYROSCOPE:
    return "rad/s";
  case ws_sensor_Type_T_LIGHT:
    return "none";
  case ws_sensor_Type_T_PRESSURE:
    return "hPa";
  case ws_sensor_Type_T_PROXIMITY:
    return "none";
  case ws_sensor_Type_T_GRAVITY:
    return "m/s^2";
  case ws_sensor_Type_T_LINEAR_ACCELERATION:
    return "m/s^2";
  case ws_sensor_Type_T_ROTATION_VECTOR:
    return "rad";
  case ws_sensor_Type_T_RELATIVE_HUMIDITY:
    return "\x25";
  case ws_sensor_Type_T_AMBIENT_TEMPERATURE:
    return "C";
  case ws_sensor_Type_T_OBJECT_TEMPERATURE:
    return "C";
  case ws_sensor_Type_T_VOLTAGE:
    return "V";
  case ws_sensor_Type_T_CURRENT:
    return "mA";
  case ws_sensor_Type_T_COLOR:
    return "none";
  case ws_sensor_Type_T_RAW:
    return "none";
  case ws_sensor_Type_T_PM10_STD:
    return "ppm";
  case ws_sensor_Type_T_PM25_STD:
    return "ppm";
  case ws_sensor_Type_T_PM100_STD:
    return "ppm";
  case ws_sensor_Type_T_PM10_ENV:
    return "ppm";
  case ws_sensor_Type_T_PM25_ENV:
    return "ppm";
  case ws_sensor_Type_T_PM100_ENV:
    return "ppm";
  case ws_sensor_Type_T_CO2:
    return "ppm";
  case ws_sensor_Type_T_GAS_RESISTANCE:
    return "\u03A9";
  case ws_sensor_Type_T_ALTITUDE:
    return "m";
  case ws_sensor_Type_T_LUX:
    return "lux";
  case ws_sensor_Type_T_ECO2:
    return "ppm";
  case ws_sensor_Type_T_UNITLESS_PERCENT:
    return "\x25";
  case ws_sensor_Type_T_AMBIENT_TEMPERATURE_FAHRENHEIT:
    return "F";
  case ws_sensor_Type_T_OBJECT_TEMPERATURE_FAHRENHEIT:
    return "F";
  case ws_sensor_Type_T_VOC_INDEX:
    return "VOC";
  case ws_sensor_Type_T_NOX_INDEX:
    return "NOX";
  case ws_sensor_Type_T_TVOC:
    return "ppb";
  case ws_sensor_Type_T_BYTES:
    return "bytes";
  case ws_sensor_Type_T_BOOLEAN:
    return "boolean";
  default:
    return "UNKNOWN";
  }
}

/*!
    @brief  Builds a JSON document for a sensor event.
    @param  doc
            The JSON document to populate.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
*/
void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, float value,
                             ws_sensor_Type read_type) {
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = "A" + String(pin);
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToSIUnit(read_type);
}

/*!
    @brief  Builds a JSON document for a sensor event.
    @param  doc
            The JSON document to populate.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
*/
void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, uint16_t value,
                             ws_sensor_Type read_type) {
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = "A" + String(pin);
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToSIUnit(read_type);
}

/*!
    @brief  Builds a JSON document for a sensor event.
    @param  doc
            The JSON document to populate.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
*/
void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, bool value,
                             ws_sensor_Type read_type) {
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = "D" + String(pin);
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToSIUnit(read_type);
}

/*!
    @brief  Logs a JSON document to the SD card.
    @param  doc
            The JSON document to log.
    @returns True if the document was successfully logged, False otherwise.
*/
bool ws_sdcard::LogJSONDoc(JsonDocument &doc) {
  size_t szJson;

#ifndef OFFLINE_MODE_DEBUG
  File32 file;
  file = _sd.open(_log_filename, O_RDWR | O_CREAT | O_AT_END);
  if (!file) {
    WS_DEBUG_PRINTLN(
        "[SD] Runtime Error: Unable to open log file for writing!");
    return false;
  }
  BufferingPrint bufferedFile(file, 64); // Add buffering to the file
  szJson = serializeJson(
      doc, file);           // Serialize the JSON to the file in 64-byte chunks
  bufferedFile.print("\n"); // JSONL format specifier
  bufferedFile.flush();     // Send the remaining bytes
  file.close();
  // Update log file's size
  _sz_cur_log_file += (szJson + 2); // +2 for newline
  // print the doc to the serial
  serializeJson(doc, Serial);
  Serial.print("\n");
#else
  szJson = serializeJson(doc, Serial);
  Serial.print("\n"); // Required JSONL format specifier
#endif

  // Do we need a new log file?
  if (_sz_cur_log_file >= _max_sz_log_file) {
    WS_DEBUG_PRINTLN(
        "[SD] NOTE: Log file has exceeded maximum size! Attempting to "
        "create a new file...");
    return CreateNewLogFile();
  }

  return true;
}

/*!
    @brief  Logs a GPIO sensor event to the SD card.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
    @returns True if the event was successfully logged, False otherwise.
*/
bool ws_sdcard::LogGPIOSensorEventToSD(uint8_t pin, float value,
                                       ws_sensor_Type read_type) {
  JsonDocument doc;
  BuildJSONDoc(doc, pin, value, read_type);
  if (!LogJSONDoc(doc))
    return false;
  return true;
}

/*!
    @brief  Logs a GPIO sensor event to the SD card.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
    @returns True if the event was successfully logged, False otherwise.
*/
bool ws_sdcard::LogGPIOSensorEventToSD(uint8_t pin, uint16_t value,
                                       ws_sensor_Type read_type) {
  JsonDocument doc;
  BuildJSONDoc(doc, pin, value, read_type);
  if (!LogJSONDoc(doc))
    return false;
  return true;
}

/*!
    @brief  Logs a GPIO sensor event to the SD card.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
    @returns True if the event was successfully logged, False otherwise.
*/
bool ws_sdcard::LogGPIOSensorEventToSD(uint8_t pin, bool value,
                                       ws_sensor_Type read_type) {
  JsonDocument doc;
  BuildJSONDoc(doc, pin, value, read_type);
  if (!LogJSONDoc(doc))
    return false;
  return true;
}

/*!
    @brief  Logs a GPIO sensor event to the SD card.
    @param  pin
            The GPIO pin number.
    @param  value
            The sensor value.
    @param  read_type
            The sensor type.
    @returns True if the event was successfully logged, False otherwise.
*/
bool ws_sdcard::LogDS18xSensorEventToSD(ws_ds18x20_Event *event_msg) {
  JsonDocument doc;
  // Iterate over the event message's sensor events
  // TODO: Standardize this Event with I2C
  for (int i = 0; i < event_msg->sensor_events_count; i++) {
    uint32_t timestamp = GetTimestamp();
    doc["timestamp"] = timestamp;
    doc["pin"] = event_msg->onewire_pin;
    doc["value"] = event_msg->sensor_events[i].value.float_value;
    doc["si_unit"] = SensorTypeToSIUnit(event_msg->sensor_events[i].type);
    LogJSONDoc(doc);
  }
  return true;
}

/*!
    @brief  Logs an I2C sensor event to the SD card.
    @param  msg_device_event
            The I2cDeviceEvent message to log.
    @returns True if the event was successfully logged, False otherwise.
*/
bool ws_sdcard::LogI2cDeviceEvent(ws_i2c_DeviceEvent *msg_device_event) {
  JsonDocument doc;
  // Pull the DeviceDescriptor out
  ws_i2c_DeviceDescriptor descriptor = msg_device_event->device_description;
  char hex_addr[5];
  snprintf(hex_addr, sizeof(hex_addr), "0x%02X", descriptor.device_address);
  doc["i2c_address"] = hex_addr;

  // Using I2C MUX?
  if (descriptor.mux_address != 0x00) {
    snprintf(hex_addr, sizeof(hex_addr), "0x%02X", descriptor.mux_address);
    doc["i2c_mux_addr"] = hex_addr;
    doc["i2c_mux_ch"] = descriptor.mux_channel;
  }

  // Log each event
  for (pb_size_t i = 0; i < msg_device_event->device_events_count; i++) {
    doc["timestamp"] = GetTimestamp();
    doc["value"] = msg_device_event->device_events[i].value.float_value;
    doc["si_unit"] =
        SensorTypeToSIUnit(msg_device_event->device_events[i].type);
    if (!LogJSONDoc(doc))
      return false;
  }
  return true;
}

#ifdef OFFLINE_MODE_DEBUG
/*!
    @brief  Waits for a valid JSON string to be received via the hardware's
            serial input or from a hardcoded test JSON string.
    @returns True if a valid JSON string was received, False otherwise.
*/
void ws_sdcard::waitForSerialConfig() {
  json_test_data = "{"
                   "\"exportVersion\": \"1.0.0\","
                   "\"exportedBy\": \"tester\","
                   "\"exportedAt\": \"2024-10-28T18:58:23.976Z\","
                   "\"exportedFromDevice\": {"
                   "\"board\": \"metroesp32s3\","
                   "\"firmwareVersion\": \"1.0.0-beta.93\","
                   "\"referenceVoltage\": 2.6,"
                   "\"totalGPIOPins\": 11,"
                   "\"totalAnalogPins\": 6"
                   "},"
                   "\"components\": ["
                   "{"
                   "\"componentAPI\": \"analogio\","
                   "\"name\": \"Analog Pin\","
                   "\"pinName\": \"D14\","
                   "\"type\": \"analog_pin\","
                   "\"mode\": \"ANALOG\","
                   "\"direction\": \"INPUT\","
                   "\"sampleMode\": \"TIMER\","
                   "\"analogReadMode\": \"PIN_VALUE\","
                   "\"period\": 5,"
                   "\"isPin\": true"
                   "},"
                   "{"
                   "\"componentAPI\": \"analogio\","
                   "\"name\": \"Analog Pin\","
                   "\"pinName\": \"D27\","
                   "\"type\": \"analog_pin\","
                   "\"mode\": \"ANALOG\","
                   "\"direction\": \"INPUT\","
                   "\"sampleMode\": \"TIMER\","
                   "\"analogReadMode\": \"PIN_VALUE\","
                   "\"period\": 5,"
                   "\"isPin\": true"
                   "},"
                   "{"
                   "\"componentAPI\": \"digitalio\","
                   "\"name\": \"Button (D4)\","
                   "\"pinName\": \"D4\","
                   "\"type\": \"push_button\","
                   "\"mode\": \"DIGITAL\","
                   "\"sampleMode\": \"EVENT\","
                   "\"direction\": \"INPUT\","
                   "\"period\": 5,"
                   "\"pull\": \"UP\","
                   "\"isPin\": true"
                   "},"
                   "{"
                   "\"componentAPI\": \"ds18x20\","
                   "\"name\": \"DS18B20: Temperature Sensor (°F)\","
                   "\"sensorTypeCount\": 2,"
                   "\"sensorType1\": \"object-temp-fahrenheit\","
                   "\"sensorType2\": \"object-temp\","
                   "\"pinName\": \"D12\","
                   "\"sensorResolution\": 12,"
                   "\"period\": 5"
                   "},"
                   "{"
                   "\"componentAPI\": \"ds18x20\","
                   "\"name\": \"DS18B20: Temperature Sensor (°F)\","
                   "\"sensorTypeCount\": 2,"
                   "\"sensorType1\": \"object-temp-fahrenheit\","
                   "\"sensorType2\": \"object-temp\","
                   "\"pinName\": \"D25\","
                   "\"sensorResolution\": 12,"
                   "\"period\": 5"
                   "}"
                   "]"
                   "}\\n\r\n";

  _serialInput = ""; // Clear the serial input buffer
  if (!_use_test_data) {
    WS_DEBUG_PRINTLN("[SD] Waiting for incoming JSON string...");
    while (true) {
      // Check if there is data available to read
      if (Serial.available() > 0) {
        char c = Serial.read();
        _serialInput += c;
        if (_serialInput.endsWith("\\n")) {
          break;
        }
      }
    }
  }
  // Remove the newline
  _serialInput.trim();
  WS_DEBUG_PRINTLN("[SD] JSON string received!");
}
#endif