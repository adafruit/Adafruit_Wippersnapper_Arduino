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
  mode_offline = false;
  _wokwi_runner = false;
  _use_test_data = false;
}

/**************************************************************************/
/*!
    @brief    Destructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::~ws_sdcard() {
  // TODO: Close any open files
  // Then, end the SD card (ends SPI transaction)
  if (mode_offline) {
    _sd.end();
    mode_offline = false;
  }
}

bool ws_sdcard::InitSDCard() {
#ifdef SD_CS_PIN
  // Attempt to initialize the SD card
  if (_sd.begin(SD_CS_PIN)) {
    mode_offline = true;
  }
#endif
  return mode_offline;
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
  WS_DEBUG_PRINTLN("[SD] Enabled DS3231 RTC");
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
  if (strcmp(rtc_type, "DS1307")) {
    did_init = InitDS1307();
    WS_DEBUG_PRINTLN("[SD] Enabled DS1307 RTC");
  } else if (strcmp(rtc_type, "DS3231")) {
    did_init = InitDS3231();
    WS_DEBUG_PRINTLN("[SD] Enabled DS3231 RTC");
  } else if (strcmp(rtc_type, "PCF8523")) {
    did_init = InitPCF8523();
    WS_DEBUG_PRINTLN("[SD] Enabled PCF8523 RTC");
  } else if (strcmp(rtc_type, "SOFT_RTC")) {
    did_init = InitSoftRTC();
    WS_DEBUG_PRINTLN("[SD] Enabled software RTC");
  } else {
    WS_DEBUG_PRINTLN(
        "[SD] FATAL Parsing error - Unknown RTC type found in JSON string!");
    did_init = false;
  }

  return did_init;
}

void ws_sdcard::CheckIn(uint8_t max_digital_pins, uint8_t max_analog_pins,
                        float ref_voltage) {
  WsV2.digital_io_controller->SetMaxDigitalPins(max_digital_pins);
  WsV2.analogio_controller->SetTotalAnalogPins(max_analog_pins);
  WsV2.analogio_controller->SetRefVoltage(ref_voltage);
}

bool ws_sdcard::ParseDigitalIOAdd(
    wippersnapper_digitalio_DigitalIOAdd &msg_DigitalIOAdd, const char *pin,
    float period, bool value, const char *sample_mode, const char *direction,
    const char *pull) {
  bool rc = true;
  strcpy(msg_DigitalIOAdd.pin_name, pin);
  msg_DigitalIOAdd.period = period;
  msg_DigitalIOAdd.value = value;
  // Determine the sample mode
  if (strcmp(sample_mode, "TIMER") == 0) {
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
  if (strcmp(direction, "INPUT") == 0) {
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

wippersnapper_sensor_SensorType
ws_sdcard::ParseSensorType(const char *sensor_type) {
  if (strcmp(sensor_type, "PIN_VALUE") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  } else if (strcmp(sensor_type, "VOLTAGE") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  } else if (strcmp(sensor_type, "ambient-temp-fahrenheit") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
  } else if (strcmp(sensor_type, "ambient-temp") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
  } else {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED;
  }
}

bool ws_sdcard::ParseAnalogIOAdd(
    wippersnapper_analogio_AnalogIOAdd &msg_AnalogIOAdd, const char *pin,
    float period, const char *mode) {
  strcpy(msg_AnalogIOAdd.pin_name, pin);
  msg_AnalogIOAdd.period = period;
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
    char *sensor_type_2) {
  strcpy(msg_DS18X20Add.onewire_pin, pin);
  msg_DS18X20Add.sensor_resolution = resolution;
  msg_DS18X20Add.period = period;
  msg_DS18X20Add.sensor_types_count = num_sensors;

  WS_DEBUG_PRINT("[SD] msg_DS18X20Add.sensor_types_count: ");
  WS_DEBUG_PRINTLN(msg_DS18X20Add.sensor_types_count);

  // Parse the first sensor type
  msg_DS18X20Add.sensor_types[0] = ParseSensorType(sensor_type_1);
  // Parse the second sensor type, if it exists
  if (num_sensors == 2) {
    msg_DS18X20Add.sensor_types[1] = ParseSensorType(sensor_type_2);
  }
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
  File32 file_config;
  JsonDocument doc;
  int max_json_len =
      2048; // TODO: It's possible this is too small - what's the max size?

  // Attempt to open and deserialize the JSON config file
  DeserializationError error;
#ifdef OFFLINE_MODE_DEBUG
  // Test Mode - do not use the SD card, use test data instead!
  if (_use_test_data == true) {
    WS_DEBUG_PRINTLN("[SD] Using SERIAL INPUT for JSON config...");
    error = deserializeJson(doc, _serialInput.c_str(), max_json_len);
  } else {
    WS_DEBUG_PRINTLN("[SD] Using TEST DATA for JSON config...");
    error = deserializeJson(doc, json_test_data, max_json_len);
  }
#else
  WS_DEBUG_PRINTLN("[SD] Opening config.json FILE...");
  file_config = _sd.open("config.json", FILE_READ);
  // TODO: Unimplemented
  // error = deserializeJson(doc, file_config, max_json_len);
#endif

  // If the JSON document failed to deserialize - halt the running device and
  // print the error because it is not possible to continue running in offline
  // mode without a valid config file
  if (error) {
    WS_DEBUG_PRINTLN("[SD] Unable to deserialize config JSON, error code: " +
                     String(error.c_str()));
    return false;
  }

  // NOTE: This is only used by the CI runner, production builds do not run
  // this!
  const char *exportedBy = doc["exportedBy"];
  if (strcmp(exportedBy, "wokwi") == 0) {
    _wokwi_runner = true;
  }

  // Parse the exportedFromDevice array
  JsonObject exportedFromDevice = doc["exportedFromDevice"];
  if (exportedFromDevice.isNull()) {
    WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - No exportedFromDevice object "
                     "found in JSON string! Unable to configure hardware!");
    return false;
  }

  // Mock the check-in process using the JSON's values
  CheckIn(exportedFromDevice["totalGPIOPins"],
          exportedFromDevice["totalAnalogPins"],
          exportedFromDevice["referenceVoltage"]);

  // Initialize and configure RTC
  const char *json_rtc = exportedFromDevice["rtc"];
  if (json_rtc == nullptr) {
    WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - rtc field not found in "
                     "configuration JSON, unable to configure RTC!");
    return false;
  }
  if (!ConfigureRTC(json_rtc)) {
    WS_DEBUG_PRINTLN("[SD] Failed to to configure RTC!");
    return false;
  }

  // Parse the "components" array into a JsonObject
  JsonArray components_ar = doc["components"].as<JsonArray>();
  if (components_ar.isNull()) {
    WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - No components array found in "
                     "JSON string!");
    return false;
  }
  int count = components_ar.size();
  WS_DEBUG_PRINTLN("[SD] Found " + String(count) + " components in JSON file!");

  // Parse each component from JSON->PB and push into a shared buffer
  for (JsonObject component : doc["components"].as<JsonArray>()) {
    wippersnapper_signal_BrokerToDevice msg_signal_b2d =
        wippersnapper_signal_BrokerToDevice_init_default;

    // Parse the component API type
    const char *component_api_type = component["componentAPI"];
    if (component_api_type == nullptr) {
      WS_DEBUG_PRINTLN("[SD] FATAL Parsing error - No component API type found "
                       "in JSON string!");
      return false;
    }

    // Determine the component type and parse it into a PB message
    if (strcmp(component_api_type, "digitalio") == 0) {
      WS_DEBUG_PRINTLN(
          "[SD] DigitalIO component found, decoding JSON to PB...");

      // Parse: JSON->DigitalIOAdd
      wippersnapper_digitalio_DigitalIOAdd msg_DigitalIOAdd =
          wippersnapper_digitalio_DigitalIOAdd_init_default;
      if (!ParseDigitalIOAdd(msg_DigitalIOAdd, component["pinName"],
                             component["period"], component["value"],
                             component["sampleMode"], component["direction"],
                             component["pull"])) {
        WS_DEBUG_PRINTLN(
            "[SD] FATAL Parsing error - Unable to parse DigitalIO component!");
        return false;
      }

      // Configure the signal message for the digitalio payload
      msg_signal_b2d.which_payload =
          wippersnapper_signal_BrokerToDevice_digitalio_add_tag;
      msg_signal_b2d.payload.digitalio_add = msg_DigitalIOAdd;
    } else if (strcmp(component_api_type, "analogio") == 0) {
      WS_DEBUG_PRINTLN("[SD] AnalogIO component found, decoding JSON to PB...");
      wippersnapper_analogio_AnalogIOAdd msg_AnalogIOAdd =
          wippersnapper_analogio_AnalogIOAdd_init_default;

      // Parse: JSON->AnalogIOAdd
      if (!ParseAnalogIOAdd(msg_AnalogIOAdd, component["pinName"],
                            component["period"], component["analogReadMode"])) {
        WS_DEBUG_PRINTLN(
            "[SD] FATAL Parsing error - Unable to parse AnalogIO component!");
        return false;
      }

      msg_signal_b2d.which_payload =
          wippersnapper_signal_BrokerToDevice_analogio_add_tag;
      msg_signal_b2d.payload.analogio_add = msg_AnalogIOAdd;
    } else if (strcmp(component_api_type, "ds18x20") == 0) {
      WS_DEBUG_PRINTLN("[SD] ds18x20 component found, decoding JSON to PB...");
      // Create new DS18X20Add message
      wippersnapper_ds18x20_Ds18x20Add msg_DS18X20Add =
          wippersnapper_ds18x20_Ds18x20Add_init_default;

      // Parse: JSON->DS18X20Add
      if (!ParseDS18X20Add(msg_DS18X20Add, component["pinName"],
                           component["sensorResolution"], component["period"],
                           component["sensorTypeCount"],
                           component["sensorType1"],
                           component["sensorType2"])) {
        WS_DEBUG_PRINTLN(
            "[SD] FATAL Parsing error - Unable to parse DS18X20 component!");
        return false;
      }

      // Configure the signal message for the ds18x20 payload
      msg_signal_b2d.which_payload =
          wippersnapper_signal_BrokerToDevice_ds18x20_add_tag;
      msg_signal_b2d.payload.ds18x20_add = msg_DS18X20Add;
    } else {
      // Unknown component API type
      WS_DEBUG_PRINTLN("[SD] Unknown component API type found: " +
                       String(component_api_type));
      return false;
    }

    // Create a temporary buffer to hold the encoded signal message
    std::vector<uint8_t> tempBuf(128);
    size_t tempBufSz;

    // Get the encoded size of the signal message first so we can resize the
    // buffer prior to encoding
    WS_DEBUG_PRINTLN("Encoding D2b signal message...");
    if (!pb_get_encoded_size(&tempBufSz,
                             wippersnapper_signal_BrokerToDevice_fields,
                             &msg_signal_b2d)) {
      WS_DEBUG_PRINTLN("[SD] ERROR: Unable to get signal message size!");
      return false;
    }
    WS_DEBUG_PRINTLN("Signal message size: " + String(tempBufSz));
    // Encode and push the signal message to the shared config buffer
    tempBuf.resize(tempBufSz);
    pb_ostream_t ostream =
        pb_ostream_from_buffer(tempBuf.data(), tempBuf.size());
    if (!ws_pb_encode(&ostream, wippersnapper_signal_BrokerToDevice_fields,
                      &msg_signal_b2d)) {
      WS_DEBUG_PRINTLN("[SD] ERROR: Unable to encode D2B signal message!");
      return false;
    }
    WsV2._sharedConfigBuffers.push_back(std::move(tempBuf));
    WS_DEBUG_PRINTLN("Encoded the D2b signal message");
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Validates a JSON string.
    @param  input
            A JSON string to validate.
    @returns True if the provided JSON string is valid, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::validateJson(const char *input) {
  JsonDocument doc, filter;

  DeserializationError error =
      deserializeJson(doc, input, DeserializationOption::Filter(filter));
  return error == DeserializationError::Ok;
}

/**************************************************************************/
/*!
    @brief  Waits for a valid JSON string to be received via the hardware's
            serial input or from a hardcoded test JSON string.
    @returns True if a valid JSON string was received, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::waitForSerialConfig() {

  // We provide three ways to use this function:
  // 1. Use a SD card with a JSON config file
  // 2. Provide a JSON string via the hardware's serial input
  // 3. Use a test JSON string - for debugging purposes ONLY

  _use_test_data = true;
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
                   "\"sensorType1\": \"ambient-temp-fahrenheit\","
                   "\"sensorType2\": \"ambient-temp\","
                   "\"pinName\": \"D12\","
                   "\"sensorResolution\": 12,"
                   "\"period\": 5"
                   "},"
                   "{"
                   "\"componentAPI\": \"ds18x20\","
                   "\"name\": \"DS18B20: Temperature Sensor (°F)\","
                   "\"sensorTypeCount\": 2,"
                   "\"sensorType1\": \"ambient-temp-fahrenheit\","
                   "\"sensorType2\": \"ambient-temp\","
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
        // Read and append to _serialInput
        char c = Serial.read();
        _serialInput += c;

        // DEBUG - Check JSON output as an Int and total output
        // WS_DEBUG_PRINT("[SD] Character read: ");
        // WS_DEBUG_PRINTLN((int)c);
        // WS_DEBUG_PRINTLN(_serialInput);

        // Check for end of JSON string using \n sequence
        if (_serialInput.endsWith("\\n")) {
          WS_DEBUG_PRINTLN("[SD] End of JSON string detected!");
          break;
        }
      }
    }
  }

  // Strip the '\n' off the end of _serialInput
  _serialInput.trim();

  // Print out the received JSON string
  WS_DEBUG_PRINT("[SD][Debug] JSON string received!");
  if (_use_test_data) {
    WS_DEBUG_PRINTLN("[from json test data]");
    WS_DEBUG_PRINTLN(json_test_data);
  } else {
    WS_DEBUG_PRINTLN(_serialInput);
  }

  // Attempt to validate the string as JSON
  if (!_use_test_data) {
    if (!validateJson(_serialInput.c_str())) {
      WS_DEBUG_PRINTLN("[SD] Invalid JSON string received!");
      return false;
    }
  } else {
    if (!validateJson(json_test_data)) {
      WS_DEBUG_PRINTLN("[SD] Invalid JSON string received!");
      return false;
    }
  }

  WS_DEBUG_PRINTLN("[SD] Valid JSON string received!");
  return true;
}

/**************************************************************************/
/*!
    @brief  Obtains a timestamp from the hardware (or software) RTC.
    @returns The current timestamp, in unixtime format.
*/
/**************************************************************************/
uint32_t ws_sdcard::GetTimestamp() {
  // Obtain RTC timestamp (TODO - refactor this out)
  DateTime now;
  if (_rtc_ds3231 != nullptr)
    now = _rtc_ds3231->now();
  else if (_rtc_ds1307 != nullptr)
    now = _rtc_ds1307->now();
  else if (_rtc_pcf8523 != nullptr)
    now = _rtc_pcf8523->now();
  else {
    // TODO! implement software millis() version of now() and unixtime()
  }

  if (_wokwi_runner)
    return 0;

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
  // Get the pin name in the correct format ("A0", "A1", etc.)
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  // Get the RTC's timestamp
  uint32_t timestamp = GetTimestamp();

  // Create the JSON document
  JsonDocument doc;

  doc["timestamp"] = timestamp;
  doc["pin"] = c_pin_name;
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToString(read_type);
  serializeJson(doc, Serial);
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
  // Get the pin name in the correct format ("A0", "A1", etc.)
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  // Get the RTC's timestamp
  uint32_t timestamp = GetTimestamp();

  // Append to the file in JSONL format
  JsonDocument doc;
  doc["timestamp"] = timestamp;
  doc["pin"] = c_pin_name;
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToString(read_type);
  serializeJson(doc, Serial);
  Serial.println(""); // JSON requires a newline at the end of each log line
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
  // Get the pin name in the correct format ("A0", "A1", etc.)
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  // Get the RTC's timestamp
  uint32_t timestamp = GetTimestamp();

  // Create the JSON document
  JsonDocument doc;
  doc["timestamp"] = timestamp;
  doc["pin"] = c_pin_name;
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToString(read_type);
  serializeJson(doc, Serial);
  Serial.println("");
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
  // Get the RTC's timestamp
  uint32_t timestamp = GetTimestamp();

  // Create the JSON document
  JsonDocument doc;
  // Iterate over the event message's sensor events
  for (int i = 0; i < event_msg->sensor_events_count; i++) {
    doc["timestamp"] = timestamp;
    doc["pin"] = event_msg->onewire_pin;
    doc["value"] = event_msg->sensor_events[i].value.float_value;
    doc["si_unit"] = SensorTypeToString(event_msg->sensor_events[i].type);
    serializeJson(doc, Serial);
    Serial.println("");
  }
  return true;
}