/*!
 * @file ws_sdcard.cpp
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
#include "ws_sdcard.h"

/**************************************************************************/
/*!
    @brief    Constructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::ws_sdcard() {
  is_mode_offline = false;
  _use_test_data = false;
  _sz_cur_log_file = 0;
}

/**************************************************************************/
/*!
    @brief    Destructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::~ws_sdcard() {
  if (is_mode_offline) {
    _sd.end();               // Close the SD card
    is_mode_offline = false; // Disable offline mode
  }
}

/**************************************************************************/
/*!
    @brief    Attempts to initialize the SD card and filesystem.
    @returns  True if the SD card was successfully initialized, False
              otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitSDCard() {
  is_mode_offline = false;
  csd_t csd;
  if (WsV2.pin_sd_cs == PIN_SD_CS_ERROR)
    return is_mode_offline;

  if (!_sd.begin(WsV2.pin_sd_cs)) {
    WS_DEBUG_PRINTLN(
        "SD initialization failed.\nDo not reformat the card!\nIs the card "
        "correctly inserted?\nIs there a wiring/soldering problem\n");
    return is_mode_offline;
  }

  // Calculate the maximum number of log files that can be stored on the SD card
  if (!_sd.card()->readCSD(&csd)) {
    WS_DEBUG_PRINTLN("ERROR: Could not read sdcard information");
    return is_mode_offline;
  }
  // get the complete sdcard capacity in bytes
  _sd_capacity = (uint64_t)512 * csd.capacity();
  // account for 3-5% fatfs overhead utilization
  size_t sd_capacity_usable = _sd_capacity * (1 - 0.05);
  // proportionally set sz of each log file to 10% of the SD card's usable capacity
  _max_sz_log_file = sd_capacity_usable / 10;
  // Regardless of sd card size, cap log files to 512MB
  if (_max_sz_log_file > MAX_SZ_LOG_FILE) {
    _max_sz_log_file = MAX_SZ_LOG_FILE;
  }
  _sd_max_num_log_files = sd_capacity_usable / _max_sz_log_file;

  is_mode_offline = true;
  return is_mode_offline;
}

/**************************************************************************/
/*!
    @brief    Initializes a DS1307 RTC
    @returns  True if the RTC was successfully initialized, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitDS1307() {
  _rtc_ds1307 = new RTC_DS1307();
  if (!_rtc_ds1307->begin()) {
    WS_DEBUG_PRINTLN("[SD] Failed to initialize DS1307 RTC");
    delete _rtc_ds1307;
    return false;
  }
  if (!_rtc_ds1307->isrunning())
    _rtc_ds1307->adjust(DateTime(F(__DATE__), F(__TIME__)));
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes a DS3231 RTC.
    @returns  True if the RTC was successfully initialized, False
              otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitDS3231() {
  _rtc_ds3231 = new RTC_DS3231();
  if (!_rtc_ds3231->begin()) {
    WS_DEBUG_PRINTLN("[SD] Failed to initialize DS3231 RTC");
    delete _rtc_ds3231;
    return false;
  }
  if (_rtc_ds3231->lostPower())
    _rtc_ds3231->adjust(DateTime(F(__DATE__), F(__TIME__)));
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes a PCF8523 RTC.
    @returns  True if the RTC was successfully initialized, False
              otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitPCF8523() {
  _rtc_pcf8523 = new RTC_PCF8523();
  if (!_rtc_pcf8523->begin()) {
    WS_DEBUG_PRINTLN("[SD] Failed to initialize PCF8523 RTC");
    delete _rtc_pcf8523;
    return false;
  }
  if (_rtc_pcf8523->lostPower())
    _rtc_pcf8523->adjust(DateTime(F(__DATE__), F(__TIME__)));
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes a software RTC.
    @returns  True if the RTC was successfully initialized, False
              otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitSoftRTC() {
  _rtc_soft->begin(DateTime(F(__DATE__), F(__TIME__)));
  return true;
}

/**************************************************************************/
/*!
    @brief  Initializes and configures a RTC for logging.
    @param  rtc_type
            The desired type of RTC to configure.
    @returns True if the RTC was successfully configured, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::ConfigureRTC(const char *rtc_type) {
  bool did_init = false;
  // Initialize the RTC based on the rtc_type
  if (strcmp(rtc_type, "DS1307") == 0) {
    did_init = InitDS1307();
    WS_DEBUG_PRINTLN("[SD] Enabled DS1307 RTC");
  } else if (strcmp(rtc_type, "DS3231") == 0) {
    did_init = InitDS3231();
    WS_DEBUG_PRINTLN("[SD] Enabled DS3231 RTC");
  } else if (strcmp(rtc_type, "PCF8523") == 0) {
    did_init = InitPCF8523();
    WS_DEBUG_PRINTLN("[SD] Enabled PCF8523 RTC");
  } else if (strcmp(rtc_type, "SOFT_RTC") == 0) {
    did_init = InitSoftRTC();
    WS_DEBUG_PRINTLN("[SD] Enabled software RTC");
  } else {
    WS_DEBUG_PRINTLN(
        "[SD] FATAL Parsing error - Unknown RTC type found in JSON string!");
    did_init = false;
  }

  return did_init;
}

/**************************************************************************/
/*!
    @brief  Configure's the hardware from the JSON's exportedFromDevice
            object.
    @param  max_digital_pins
            The total number of digital pins on the device.
    @param  max_analog_pins
            The total number of analog pins on the device.
    @param  ref_voltage
            The reference voltage of the device, in Volts.
*/
/**************************************************************************/
void ws_sdcard::CheckIn(uint8_t max_digital_pins, uint8_t max_analog_pins,
                        float ref_voltage) {
  WsV2.digital_io_controller->SetMaxDigitalPins(max_digital_pins);
  WsV2.analogio_controller->SetTotalAnalogPins(max_analog_pins);
  WsV2.analogio_controller->SetRefVoltage(ref_voltage);
}

/**************************************************************************/
/*!
    @brief  Parses a sensor type from the JSON configuration file.
    @param  sensor_type
            The sensor type to parse.
    @returns The parsed sensor type.
*/
/**************************************************************************/
wippersnapper_sensor_SensorType
ws_sdcard::ParseSensorType(const char *sensor_type) {
  if (strcmp(sensor_type, "PIN_VALUE") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  } else if (strcmp(sensor_type, "VOLTAGE") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  } else if (strcmp(sensor_type, "object-temp-fahrenheit") == 0) {
    WS_DEBUG_PRINTLN("Found object-temp-fahrenheit");
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT;
  } else if (strcmp(sensor_type, "object-temp") == 0) {
    WS_DEBUG_PRINTLN("Found object-temp");
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE;
  } else {
    WS_DEBUG_PRINT("Found unspecified sensortype - ");
    WS_DEBUG_PRINTLN(sensor_type);
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED;
  }
}

bool ws_sdcard::ValidateJSONKey(const char *key, const char *error_msg) {
  if (strcmp(key, UNKNOWN_VALUE) == 0) {
    WS_DEBUG_PRINTLN(error_msg);
    return false;
  }
  return true;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::ParseDigitalIOAdd(
    wippersnapper_digitalio_DigitalIOAdd &msg_DigitalIOAdd, const char *pin,
    float period, bool value, const char *sample_mode, const char *direction,
    const char *pull) {
  bool rc = true;
  if (!ValidateJSONKey(pin, "[SD] Parsing Error: Digital pin name not found!"))
    return false;
  strcpy(msg_DigitalIOAdd.pin_name, pin);

  if (period == 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Digital pin period not found!");
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
    msg_DigitalIOAdd.sample_mode =
        wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_TIMER;
  } else if (strcmp(sample_mode, "EVENT") == 0) {
    msg_DigitalIOAdd.sample_mode =
        wippersnapper_digitalio_DigitalIOSampleMode_DIGITAL_IO_SAMPLE_MODE_EVENT;
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
      msg_DigitalIOAdd.gpio_direction =
          wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT_PULL_UP;
    } else {
      msg_DigitalIOAdd.gpio_direction =
          wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT;
    }
  } else if (strcmp(direction, "OUTPUT") == 0) {
    WS_DEBUG_PRINTLN(
        "[SD] Error - Can not set OUTPUT direction in offline mode!");
    rc = false;
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown direction found: " +
                     String(direction));
    rc = false;
  }
  return rc;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::ParseAnalogIOAdd(
    wippersnapper_analogio_AnalogIOAdd &msg_AnalogIOAdd, const char *pin,
    float period, const char *mode) {

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
  if (msg_AnalogIOAdd.read_mode ==
      wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown read mode found: " +
                     String(mode));
    return false;
  }
  return true;
}

bool ws_sdcard::ParseDS18X20Add(
    wippersnapper_ds18x20_Ds18x20Add &msg_DS18X20Add, const char *pin,
    int resolution, float period, int num_sensors, const char *sensor_type_1,
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

  WS_DEBUG_PRINT("[SD] msg_DS18X20Add.sensor_types_count: ");
  WS_DEBUG_PRINTLN(msg_DS18X20Add.sensor_types_count);

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

/**************************************************************************/
/*!
    @brief  Pushes a signal message to the shared buffer.
    @param  msg_signal
            The signal message to push.
    @returns True if the signal message was successfully pushed to the shared
             buffer, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::AddSignalMessageToSharedBuffer(
    wippersnapper_signal_BrokerToDevice &msg_signal) {
  // Create a temporary buffer to hold the encoded signal message
  std::vector<uint8_t> tempBuf(512);
  size_t tempBufSz;

  // Get the encoded size of the signal message first so we can resize the
  // buffer prior to encoding
  WS_DEBUG_PRINTLN("Encoding D2b signal message...");
  if (!pb_get_encoded_size(&tempBufSz,
                           wippersnapper_signal_BrokerToDevice_fields,
                           &msg_signal)) {
    WS_DEBUG_PRINTLN("[SD] ERROR: Unable to get signal message size!");
    return false;
  }

  // Encode and push the signal message to the shared config buffer
  tempBuf.resize(tempBufSz);
  pb_ostream_t ostream = pb_ostream_from_buffer(tempBuf.data(), tempBuf.size());
  if (!ws_pb_encode(&ostream, wippersnapper_signal_BrokerToDevice_fields,
                    &msg_signal)) {
    WS_DEBUG_PRINTLN("[SD] ERROR: Unable to encode D2B signal message!");
    return false;
  }
  WsV2._sharedConfigBuffers.push_back(std::move(tempBuf));
  WS_DEBUG_PRINTLN("Encoded the D2b signal message");
  return true;
}

/**************************************************************************/
/*!
    @brief  Creates a new logging file on the SD card using the RTC's
            timestamp and sets the current log file path to reflect this
            file.
    @returns True if a log file was successfully created, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::CreateNewLogFile() {
  if (_sd_cur_log_files >= _sd_max_num_log_files) {
    WS_DEBUG_PRINTLN(
        "[SD] Maximum number of log files for SD card capacity reached!");
    return false;
  }

  File32 file;
  // Generate a name for the new log file using the RTC's timestamp
  String logFilename = "log_" + String(GetTimestamp()) + ".json";
  static char log_filename_buffer[256];
  strncpy(log_filename_buffer, logFilename.c_str(),
          sizeof(log_filename_buffer) - 1);
  log_filename_buffer[sizeof(log_filename_buffer) - 1] = '\0';
  _log_filename = log_filename_buffer;

  // Attempt to create the new log file
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

/**************************************************************************/
/*!
    @brief  Searches for and parses the JSON configuration file and sets up
            the hardware accordingly.
    @returns True if the JSON file was successfully parsed and the hardware
             was successfully configured. False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::parseConfigFile() {
  // TODO: THIS IS JUST DEBUG!
  WS_DEBUG_PRINT("SD card capacity: ");
  WS_DEBUG_PRINTLN(_sd_capacity);
  WS_DEBUG_PRINT("Maximum number of log files: ");
  WS_DEBUG_PRINTLN(_sd_max_num_log_files);
  WS_DEBUG_PRINT("Maximum size of log file: ");
  WS_DEBUG_PRINTLN(_max_sz_log_file);

  int max_json_len = 4096;
  DeserializationError error;
  JsonDocument doc;

#ifndef OFFLINE_MODE_DEBUG
  WS_DEBUG_PRINTLN("[SD] Parsing config.json...");
  doc = WsV2._config_doc; // Use the config document from the filesystem
#else
  // Use test data rather than data from the filesystem
  if (!_use_test_data) {
    WS_DEBUG_PRINTLN("[SD] Parsing Serial Input...");
    WS_DEBUG_PRINT(_serialInput);
    error = deserializeJson(doc, _serialInput.c_str(), max_json_len);
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Test Data...");
    error = deserializeJson(doc, json_test_data, max_json_len);
  }
#endif
  // It is not possible to continue running in offline mode without a valid
  // config file
  if (error) {
    WS_DEBUG_PRINTLN("[SD] Unable to deserialize config JSON, error code: " +
                     String(error.c_str()));
    return false;
  }
  WS_DEBUG_PRINTLN("[SD] Successfully deserialized JSON config file!");

  // Check the file's integrity
  if (!ValidateChecksum(doc)) {
    WS_DEBUG_PRINTLN("[SD] Checksum mismatch, file has been modified from its "
                     "original state!");
  }
  WS_DEBUG_PRINTLN("[SD] JSON checksum OK!");

  // Begin parsing the JSON document
  JsonObject exportedFromDevice = doc["exportedFromDevice"];
  if (exportedFromDevice.isNull()) {
    WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - No exportedFromDevice object "
                     "found in JSON string! Unable to configure hardware!");
    return false;
  }
  JsonArray components_ar = doc["components"].as<JsonArray>();
  if (components_ar.isNull()) {
    WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - No components array found in "
                     "JSON string!");
    return false;
  }

  // We don't talk to IO here, perform an "offline" device check-in
  CheckIn(exportedFromDevice["totalGPIOPins"] | 0,
          exportedFromDevice["totalAnalogPins"] | 0,
          exportedFromDevice["referenceVoltage"] | 0.0);
  setStatusLEDBrightness(exportedFromDevice["statusLEDBrightness"] | 0.3);

#ifndef OFFLINE_MODE_WOKWI
  const char *json_rtc = exportedFromDevice["rtc"] | "SOFT_RTC";
  if (!ConfigureRTC(json_rtc)) {
    WS_DEBUG_PRINTLN("[SD] Failed to to configure RTC!");
    return false;
  }
#else
  WS_DEBUG_PRINTLN("[SD] Skipping RTC configuration for Wokwi Simulator...");
#endif

  // Parse each component from JSON->PB and push into a shared buffer
  for (JsonObject component : doc["components"].as<JsonArray>()) {
    wippersnapper_signal_BrokerToDevice msg_signal_b2d =
        wippersnapper_signal_BrokerToDevice_init_default;

    // Parse the component API type
    const char *component_api_type = component["componentAPI"];
    if (component_api_type == nullptr) {
      WS_DEBUG_PRINTLN(
          "[SD] FATAL Parsing error - Missing component API type!");
      return false;
    }

    // Determine the component type and parse it into a PB message
    if (strcmp(component_api_type, "digitalio") == 0) {
      WS_DEBUG_PRINTLN(
          "[SD] DigitalIO component found, decoding JSON to PB...");
      wippersnapper_digitalio_DigitalIOAdd msg_DigitalIOAdd =
          wippersnapper_digitalio_DigitalIOAdd_init_default;
      if (!ParseDigitalIOAdd(
              msg_DigitalIOAdd, component["pinName"] | UNKNOWN_VALUE,
              component["period"] | 0.0, component["value"],
              component["sampleMode"] | UNKNOWN_VALUE,
              component["direction"] | UNKNOWN_VALUE, component["pull"])) {
        WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - Unable to parse "
                         "DigitalIO component!");
        return false;
      }

      msg_signal_b2d.which_payload =
          wippersnapper_signal_BrokerToDevice_digitalio_add_tag;
      msg_signal_b2d.payload.digitalio_add = msg_DigitalIOAdd;
    } else if (strcmp(component_api_type, "analogio") == 0) {
      WS_DEBUG_PRINTLN("[SD] AnalogIO component found, decoding JSON to PB...");
      wippersnapper_analogio_AnalogIOAdd msg_AnalogIOAdd =
          wippersnapper_analogio_AnalogIOAdd_init_default;
      if (!ParseAnalogIOAdd(msg_AnalogIOAdd,
                            component["pinName"] | UNKNOWN_VALUE,
                            component["period"] | 0.0,
                            component["analogReadMode"] | UNKNOWN_VALUE)) {
        WS_DEBUG_PRINTLN(
            "[SD] FATAL Parsing error - Unable to parse AnalogIO component!");
        return false;
      }

      msg_signal_b2d.which_payload =
          wippersnapper_signal_BrokerToDevice_analogio_add_tag;
      msg_signal_b2d.payload.analogio_add = msg_AnalogIOAdd;
    } else if (strcmp(component_api_type, "ds18x20") == 0) {
      WS_DEBUG_PRINTLN("[SD] Ds18x20 component found, decoding JSON to PB...");
      wippersnapper_ds18x20_Ds18x20Add msg_DS18X20Add =
          wippersnapper_ds18x20_Ds18x20Add_init_default;
      if (!ParseDS18X20Add(msg_DS18X20Add, component["pinName"] | UNKNOWN_VALUE,
                           component["sensorResolution"] | 0,
                           component["period"] | 0.0,
                           component["sensorTypeCount"] | 0,
                           component["sensorType1"] | UNKNOWN_VALUE,
                           component["sensorType2"] | UNKNOWN_VALUE)) {
        WS_DEBUG_PRINTLN(
            "[SD] FATAL Parsing error - Unable to parse DS18X20 component!");
        return false;
      }

      msg_signal_b2d.which_payload =
          wippersnapper_signal_BrokerToDevice_ds18x20_add_tag;
      msg_signal_b2d.payload.ds18x20_add = msg_DS18X20Add;
    } else {
      // Unknown component API type
      WS_DEBUG_PRINTLN("[SD] Unknown component API type found: " +
                       String(component_api_type));
      return false;
    }

    // App handles the signal messages, in-order
    if (!AddSignalMessageToSharedBuffer(msg_signal_b2d)) {
      WS_DEBUG_PRINTLN(
          "[SD] FATAL Error - Unable to add signal message to shared buffer!");
      return false;
    }
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Obtains a timestamp from the hardware (or software) RTC.
    @returns The current timestamp, in unixtime format.
*/
/**************************************************************************/
uint32_t ws_sdcard::GetTimestamp() {
  DateTime now;
  if (_rtc_ds3231 != nullptr)
    now = _rtc_ds3231->now();
  else if (_rtc_ds1307 != nullptr)
    now = _rtc_ds1307->now();
  else if (_rtc_pcf8523 != nullptr)
    now = _rtc_pcf8523->now();
  else if (_rtc_soft != nullptr) {
    now = _rtc_soft->now();
  } else { // we're either using a simulator or have undefined behavior
    return 0;
  }

  return now.unixtime();
}

/**************************************************************************/
/*!
    @brief  Converts a SensorType enum to a string.
    @param  sensorType
            The SensorType enum to convert.
    @returns A string representation of the SensorType enum.
*/
/**************************************************************************/
const char *SensorTypeToString(wippersnapper_sensor_SensorType sensorType) {
  switch (sensorType) {
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED:
    return "UNSPECIFIED";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER:
    return "ACCELEROMETER";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD:
    return "MAGNETIC_FIELD";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ORIENTATION:
    return "ORIENTATION";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_GYROSCOPE:
    return "GYROSCOPE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_LIGHT:
    return "LIGHT";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE:
    return "PRESSURE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PROXIMITY:
    return "PROXIMITY";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_GRAVITY:
    return "GRAVITY";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_LINEAR_ACCELERATION:
    return "LINEAR_ACCELERATION";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ROTATION_VECTOR:
    return "ROTATION_VECTOR";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
    return "RELATIVE_HUMIDITY";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
    return "AMBIENT_TEMPERATURE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE:
    return "OBJECT_TEMPERATURE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE:
    return "VOLTAGE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT:
    return "CURRENT";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_COLOR:
    return "COLOR";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW:
    return "RAW";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_STD:
    return "PM10_STD";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_STD:
    return "PM25_STD";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_STD:
    return "PM100_STD";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_ENV:
    return "PM10_ENV";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_ENV:
    return "PM25_ENV";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_ENV:
    return "PM100_ENV";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_CO2:
    return "CO2";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_GAS_RESISTANCE:
    return "GAS_RESISTANCE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ALTITUDE:
    return "ALTITUDE";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_LUX:
    return "LUX";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ECO2:
    return "ECO2";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_UNITLESS_PERCENT:
    return "UNITLESS_PERCENT";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT:
    return "AMBIENT_TEMPERATURE_FAHRENHEIT";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT:
    return "OBJECT_TEMPERATURE_FAHRENHEIT";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_VOC_INDEX:
    return "VOC_INDEX";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_NOX_INDEX:
    return "NOX_INDEX";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_TVOC:
    return "TVOC";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_BYTES:
    return "BYTES";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_BOOLEAN:
    return "BOOLEAN";
  default:
    return "UNKNOWN";
  }
}

void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, float value,
                             wippersnapper_sensor_SensorType read_type) {
  char pin_name[12];
  sprintf(pin_name, "A%d", pin);
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = pin_name;
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToString(read_type);
}

void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, uint16_t value,
                             wippersnapper_sensor_SensorType read_type) {
  char pin_name[12];
  sprintf(pin_name, "A%d", pin);
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = pin_name;
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToString(read_type);
}

void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, bool value,
                             wippersnapper_sensor_SensorType read_type) {
  char pin_name[12];
  sprintf(pin_name, "D%d", pin);
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = pin_name;
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToString(read_type);
}

bool ws_sdcard::LogJSONDoc(JsonDocument &doc) {
  size_t szJson;
  // Serialize the JSON document
#ifndef OFFLINE_MODE_DEBUG
  File32 file;
  file = _sd.open(_log_filename, O_RDWR | O_CREAT | O_AT_END);
  if (!file) {
    WS_DEBUG_PRINTLN(
        "[SD] FATAL Error - Unable to open the log file for writing!");
    return false;
  }
  BufferingPrint bufferedFile(file, 64); // Add buffering to the file
  szJson = serializeJson(
      doc, file);           // Serialize the JSON to the file in 64-byte chunks
  bufferedFile.print("\n"); // JSONL format specifier
  bufferedFile.flush();     // Send the remaining bytes
  // print the doc to the serial
  serializeJson(doc, Serial);
  Serial.print("\n");
#else
  szJson = serializeJson(doc, Serial); // TODO: Add buffering here, too?
  Serial.print("\n");                  // JSONL format specifier
#endif
  _sz_cur_log_file = szJson + 2; // +2 bytes for "\n"

  if (_sz_cur_log_file >= _max_sz_log_file) {
    WS_DEBUG_PRINTLN("[SD] Log file has exceeded maximum size! Attempting to "
                     "create a new file...");
    if (!CreateNewLogFile())
      return false;
    return false;
  }

  return true;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::LogGPIOSensorEventToSD(
    uint8_t pin, float value, wippersnapper_sensor_SensorType read_type) {
  JsonDocument doc;
  BuildJSONDoc(doc, pin, value, read_type);
  if (!LogJSONDoc(doc))
    return false;
  return true;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::LogGPIOSensorEventToSD(
    uint8_t pin, uint16_t value, wippersnapper_sensor_SensorType read_type) {
  JsonDocument doc;
  BuildJSONDoc(doc, pin, value, read_type);
  if (!LogJSONDoc(doc))
    return false;
  return true;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::LogGPIOSensorEventToSD(
    uint8_t pin, bool value, wippersnapper_sensor_SensorType read_type) {
  JsonDocument doc;
  BuildJSONDoc(doc, pin, value, read_type);
  if (!LogJSONDoc(doc))
    return false;
  return true;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::LogDS18xSensorEventToSD(
    wippersnapper_ds18x20_Ds18x20Event *event_msg) {
  JsonDocument doc;
  // Iterate over the event message's sensor events
  for (int i = 0; i < event_msg->sensor_events_count; i++) {
    uint32_t timestamp = GetTimestamp();
    doc["timestamp"] = timestamp;
    doc["pin"] = event_msg->onewire_pin;
    doc["value"] = event_msg->sensor_events[i].value.float_value;
    doc["si_unit"] = SensorTypeToString(event_msg->sensor_events[i].type);
    LogJSONDoc(doc);
  }
  return true;
}