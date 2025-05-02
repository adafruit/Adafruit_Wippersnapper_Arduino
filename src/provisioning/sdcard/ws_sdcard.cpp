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

/**************************************************************************/
/*!
    @brief    Initializes the SD card.
    @param    pin_cs
              The chip select pin for the SD card.
    @returns  True if the SD card was successfully initialized, False
              otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitSdCard(uint8_t pin_cs) {
#ifdef SD_USE_SPI_1
  SdSpiConfig _sd_spi_cfg(pin_cs, DEDICATED_SPI, SPI_SD_CLOCK, &SPI1);
#else
  SdSpiConfig _sd_spi_cfg(pin_cs, DEDICATED_SPI, SPI_SD_CLOCK);
#endif
  if (!_sd.begin(_sd_spi_cfg)) {
    WS_DEBUG_PRINTLN(
        "[SD] Error: SD initialization failed.\nDo not reformat the "
        "card!\nIs the card "
        "correctly inserted?\nIs there a wiring/soldering problem\n");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Constructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::ws_sdcard() {
  _use_test_data = false;
  _is_soft_rtc = false;

  _sz_cur_log_file = 0;
  _sd_cur_log_files = 0;

  // Case 1: Try to initialize the SD card with the pin from the config file
  bool did_init = false;
  if (WsV2.pin_sd_cs != SD_CS_CFG_NOT_FOUND) {
    WS_DEBUG_PRINTLN(
        "Attempting to initialize SD card with pin from config file");
    did_init = InitSdCard(WsV2.pin_sd_cs);
  }

  // Case 2: Try to initialize the SD card with the default
  // board SD CS pin (found within ws_boards.h)
  if (!did_init) {
    if (InitSdCard(SD_CS_PIN)) {
// Attempt to update the config file with the default pin
#ifndef OFFLINE_MODE_WOKWI
      did_init = WsV2._fileSystemV2->AddSDCSPinToFileConfig(SD_CS_PIN);
#else // WOKWI Test Mode
      WsV2.pin_sd_cs = 15;
      did_init = true;
#endif
    }
  }

  // If sd initialized - configure the sd card
  if (did_init)
    ConfigureSDCard();

  is_mode_offline = did_init;
}

/**************************************************************************/
/*!
    @brief    Destructs an instance of the Wippersnapper SD card class.
*/
/**************************************************************************/
ws_sdcard::~ws_sdcard() {
  if (is_mode_offline) {
    _sd.end(); // Close the SD card interface
  }
  is_mode_offline = false;
}

void ws_sdcard::ConfigureSDCard() {
  // Calculate the maximum number of log files that can be stored on the SD card
  csd_t csd;
  if (!_sd.card()->readCSD(&csd)) {
    WS_DEBUG_PRINTLN("[SD] Error: Could not read card information");
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
/**************************************************************************/
/*!
    @brief    Initializes a DS1307 RTC
    @returns  True if the RTC was successfully initialized, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitDS1307() {
  _rtc_ds1307 = new RTC_DS1307();
  if (!_rtc_ds1307->begin()) {
    if (!_rtc_ds1307->begin(&Wire1)) {
      WS_DEBUG_PRINTLN("[SD] Error: Failed to initialize DS1307 RTC");
      delete _rtc_ds1307;
      return false;
    }
  }
  if (!_rtc_ds1307->isrunning())
    _rtc_ds1307->adjust(DateTime(F(__DATE__), F(__TIME__)));
  _cfg_i2c_addresses.push_back(0x68); // Disable auto-config for DS1307
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
  WS_DEBUG_PRINTLN("Begin DS3231 init");
  _rtc_ds3231 = new RTC_DS3231();
  if (!_rtc_ds3231->begin(&Wire)) {
    if (!_rtc_ds3231->begin(&Wire1)) {
      WS_DEBUG_PRINTLN("[SD] Error: Failed to initialize DS3231 RTC");
      delete _rtc_ds3231;
      return false;
    }
  }
  if (_rtc_ds3231->lostPower())
    _rtc_ds3231->adjust(DateTime(F(__DATE__), F(__TIME__)));
  _cfg_i2c_addresses.push_back(0x68); // Disable auto-config for DS3231
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
  WsV2._i2c_controller->ScanI2cBus(true);
  WS_DEBUG_PRINT("[sd] Scanned I2C Devices: ")
  WS_DEBUG_PRINTLN(WsV2._i2c_controller->GetScanDeviceCount());
  WS_DEBUG_PRINT("Was Device Found? ");
  WS_DEBUG_PRINTLN(WsV2._i2c_controller->WasDeviceScanned(0x68));


  _rtc_pcf8523 = new RTC_PCF8523();
  if (!_rtc_pcf8523->begin(WsV2._i2c_controller->GetI2cBus())) {
    WS_DEBUG_PRINTLN("[SD] Error: Failed to initialize PCF8523 RTC on WIRE");
    if (!_rtc_pcf8523->begin(&Wire1)) {
      WS_DEBUG_PRINTLN("[SD] Error: Failed to initialize PCF8523 RTC on WIRE1");
      delete _rtc_pcf8523;
      return false;
    }
  }
  if (!_rtc_pcf8523->initialized() || _rtc_pcf8523->lostPower()) {
    _rtc_pcf8523->adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  _rtc_pcf8523->start();
  _cfg_i2c_addresses.push_back(0x68); // Disable auto-config for DS3231
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes a "soft" RTC for devices without a physical
              RTC module attached.
    @returns  True if the soft RTC was successfully initialized, False
              otherwise.
*/
/**************************************************************************/
bool ws_sdcard::InitSoftRTC() {
  _is_soft_rtc = true;
  _soft_rtc_counter = 0;
  return _is_soft_rtc;
}

/**************************************************************************/
/*!
    @brief    Increments the "soft" RTC.
*/
/**************************************************************************/
void ws_sdcard::TickSoftRTC() { _soft_rtc_counter++; }

/**************************************************************************/
/*!
    @brief    Returns the current timestamp from the RTC.
    @returns  The current timestamp from the RTC.
*/
/**************************************************************************/
uint32_t ws_sdcard::GetSoftRTCTime() { return _soft_rtc_counter; }

/**************************************************************************/
/*!
    @brief  Initializes and configures a RTC for logging.
    @param  rtc_type
            The desired type of RTC to configure.
    @returns True if the RTC was successfully configured, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::ConfigureRTC(const char *rtc_type) {
  if (strcmp(rtc_type, "DS1307") == 0) {
    return InitDS1307();
  } else if (strcmp(rtc_type, "DS3231") == 0) {
    return InitDS3231();
  } else if (strcmp(rtc_type, "PCF8523") == 0) {
    return InitPCF8523();
  } else if (strcmp(rtc_type, "SOFT") == 0) {
    return InitSoftRTC();
  }

  WS_DEBUG_PRINTLN("[SD] Error: Unknown RTC type found in JSON string!");
  WS_DEBUG_PRINTLN("[SD] Falling back to soft RTC...");
  return InitSoftRTC();
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
    @returns The corresponding SensorType
*/
/**************************************************************************/
wippersnapper_sensor_SensorType
ws_sdcard::ParseSensorType(const char *sensor_type) {
  if (strcmp(sensor_type, "raw") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  } else if (strcmp(sensor_type, "voltage") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE;
  } else if (strcmp(sensor_type, "current") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT;
  } else if (strcmp(sensor_type, "object-temp-fahrenheit") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT;
  } else if (strcmp(sensor_type, "object-temp") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE;
  } else if (strcmp(sensor_type, "ambient-temp") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
  } else if (strcmp(sensor_type, "ambient-temp-fahrenheit") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
  } else if (strcmp(sensor_type, "accelerometer") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER;
  } else if (strcmp(sensor_type, "magnetic-field") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD;
  } else if (strcmp(sensor_type, "orientation") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_ORIENTATION;
  } else if (strcmp(sensor_type, "gyroscope") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_GYROSCOPE;
  } else if (strcmp(sensor_type, "gravity") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_GRAVITY;
  } else if (strcmp(sensor_type, "linear-acceleration") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_LINEAR_ACCELERATION;
  } else if (strcmp(sensor_type, "rotation-vector") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_ROTATION_VECTOR;
  } else if (strcmp(sensor_type, "altitude") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_ALTITUDE;
  } else if (strcmp(sensor_type, "relative-humidity") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
  } else if (strcmp(sensor_type, "pressure") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE;
  } else if (strcmp(sensor_type, "light") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_LIGHT;
  } else if (strcmp(sensor_type, "lux") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_LUX;
  } else if (strcmp(sensor_type, "proximity") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PROXIMITY;
  } else if (strcmp(sensor_type, "pm10-std") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_STD;
  } else if (strcmp(sensor_type, "pm25-std") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_STD;
  } else if (strcmp(sensor_type, "pm100-std") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_STD;
  } else if (strcmp(sensor_type, "pm10-env") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_ENV;
  } else if (strcmp(sensor_type, "pm25-env") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_ENV;
  } else if (strcmp(sensor_type, "pm100-env") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_ENV;
  } else if (strcmp(sensor_type, "co2") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_CO2;
  } else if (strcmp(sensor_type, "eco2") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_ECO2;
  } else if (strcmp(sensor_type, "gas-resistance") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_GAS_RESISTANCE;
  } else if (strcmp(sensor_type, "voc-index") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_VOC_INDEX;
  } else if (strcmp(sensor_type, "nox-index") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_NOX_INDEX;
  } else if (strcmp(sensor_type, "tvoc") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_TVOC;
  } else if (strcmp(sensor_type, "color") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_COLOR;
  } else if (strcmp(sensor_type, "unitless-percent") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_UNITLESS_PERCENT;
  } else if (strcmp(sensor_type, "bytes") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_BYTES;
  } else if (strcmp(sensor_type, "boolean") == 0) {
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_BOOLEAN;
  } else {
    WS_DEBUG_PRINT("[SD] ERROR: Found unspecified SensorType - ");
    WS_DEBUG_PRINTLN(sensor_type);
    return wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED;
  }
}

bool ws_sdcard::ParseDigitalIOAdd(
    JsonObject &component,
    wippersnapper_digitalio_DigitalIOAdd &msg_DigitalIOAdd) {
  strcpy(msg_DigitalIOAdd.pin_name, component["pinName"] | UNKNOWN_VALUE);
  msg_DigitalIOAdd.period = component["period"] | 0.0;
  if (msg_DigitalIOAdd.period == 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Digital pin period less than 1.0 "
                     "seconds or not found!");
    return false;
  }

  // Optionally set pin value
  const char *value = component["value"];
  if (value != nullptr) {
    msg_DigitalIOAdd.value = value;
  }

  // Optionally determine pin sampling mode
  const char *sample_mode = component["sampleMode"];
  if (sample_mode == nullptr) {
    WS_DEBUG_PRINTLN(
        "[SD] Parsing Error: Digital pin's sample mode not found!");
    return false;
  }
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

  // Determine GPIO direction and pull mode
  const char *direction = component["direction"];
  if (direction == nullptr) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Digital pin direction not found!");
    return false;
  }
  const char *pull = component["pull"];

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
        "[SD] NotImplementedError - OUTPUT direction not supported!");
    return false;
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown GPIO direction found: " +
                     String(direction));
    return false;
  }

  return true;
}

bool ws_sdcard::ParseAnalogIOAdd(
    JsonObject &component,
    wippersnapper_analogio_AnalogIOAdd &msg_AnalogIOAdd) {
  strcpy(msg_AnalogIOAdd.pin_name, component["pinName"] | UNKNOWN_VALUE);
  msg_AnalogIOAdd.period = component["period"] | 0.0;
  if (msg_AnalogIOAdd.period < 1.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Analog pin period less than 1.0 "
                     "seconds or not found!");
    return false;
  }
  msg_AnalogIOAdd.read_mode =
      ParseSensorType(component["mode"] | "UNSPECIFIED");
  if (msg_AnalogIOAdd.read_mode ==
      wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Unknown read mode found: " +
                     String(component["mode"]));
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Converts a string-encoded hex value to an integer.
    @param  hex_str
            The string-encoded hex value to convert.
    @returns The integer value of the hex string.
*/
/**************************************************************************/
uint32_t ws_sdcard::HexStrToInt(const char *hex_str) {
  return std::stoi(hex_str, nullptr, 16);
}

/**************************************************************************/
/*!
    @brief  Parses a DS18x20Add message from the JSON configuration file.
    @param  component
            The JSON object to parse.
    @param  msg_ds18x20_add
            The DS18x20Add message to populate.
    @returns True if the DS18x20Add message was successfully parsed, False
             otherwise.
*/
/**************************************************************************/
bool ws_sdcard::ParseDS18xAdd(
    JsonObject &component, wippersnapper_ds18x20_Ds18x20Add &msg_ds18x20_add) {
  strcpy(msg_ds18x20_add.onewire_pin, component["pinName"] | UNKNOWN_VALUE);
  msg_ds18x20_add.sensor_resolution = component["sensorResolution"] | 0;
  msg_ds18x20_add.period = component["period"] | 0.0;
  msg_ds18x20_add.sensor_types_count = 0;
  for (JsonObject components_0_ds18x20SensorType :
       component["ds18x20SensorTypes"].as<JsonArray>()) {
    msg_ds18x20_add.sensor_types[msg_ds18x20_add.sensor_types_count] =
        ParseSensorType(components_0_ds18x20SensorType["type"]);
    msg_ds18x20_add.sensor_types_count++;
  }
  return true;
}

/**************************************************************************/
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
/**************************************************************************/
bool ws_sdcard::ParseI2cDeviceAddReplace(
    JsonObject &component,
    wippersnapper_i2c_I2cDeviceAddOrReplace &msg_i2c_add) {
  strcpy(msg_i2c_add.i2c_device_name,
         component["i2cDeviceName"] | UNKNOWN_VALUE);
  msg_i2c_add.i2c_device_period = component["period"] | 0.0;
  if (msg_i2c_add.i2c_device_period < 0.0) {
    WS_DEBUG_PRINTLN("[SD] Parsing Error: Invalid I2C device period!");
    return false;
  }

  msg_i2c_add.has_i2c_device_description = true;
  strcpy(msg_i2c_add.i2c_device_description.i2c_bus_scl,
         component["i2cBusScl"] | "default");
  strcpy(msg_i2c_add.i2c_device_description.i2c_bus_sda,
         component["i2cBusSda"] | "default");

  const char *addr_device = component["i2cDeviceAddress"] | "0x00";
  msg_i2c_add.i2c_device_description.i2c_device_address =
      HexStrToInt(addr_device);

  // MUXes, Seesaw, special devices should have an auto-init flag set to false
  const char *is_auto = component["autoConfig"] | "true";
  if (strcmp(is_auto, "false") == 0) {
    WS_DEBUG_PRINTLN("[SD] autoConfig = false, do not attempt to automatically "
                     "initialize this address");
    _cfg_i2c_addresses.push_back(
        msg_i2c_add.i2c_device_description.i2c_device_address);
  }

  const char *addr_mux = component["i2cMuxAddress"] | "0x00";
  msg_i2c_add.i2c_device_description.i2c_mux_address = HexStrToInt(addr_mux);

  const char *mux_channel = component["i2cMuxChannel"] | "0xFFFF";
  msg_i2c_add.i2c_device_description.i2c_mux_channel = HexStrToInt(mux_channel);

  msg_i2c_add.i2c_device_sensor_types_count = 0;
  for (JsonObject components_0_i2cDeviceSensorType :
       component["i2cDeviceSensorTypes"].as<JsonArray>()) {
    msg_i2c_add
        .i2c_device_sensor_types[msg_i2c_add.i2c_device_sensor_types_count] =
        ParseSensorType(components_0_i2cDeviceSensorType["type"]);
    msg_i2c_add.i2c_device_sensor_types_count++;
  }

  return true;
}

void ws_sdcard::ParseI2cAddScanned(
    wippersnapper_i2c_I2cDeviceAddOrReplace &msg_i2c_add_scanned,
    size_t scan_result_idx) {
  uint32_t addr_device =
      WsV2._i2c_controller->GetScanDeviceAddress(scan_result_idx);
  msg_i2c_add_scanned.i2c_device_description.i2c_device_address = addr_device;
  strcpy(msg_i2c_add_scanned.i2c_device_name, UNKNOWN_DRIVER_NAME);
  msg_i2c_add_scanned.i2c_device_period = DEFAULT_SENSOR_PERIOD;
  msg_i2c_add_scanned.has_i2c_device_description = true;
  strcpy(msg_i2c_add_scanned.i2c_device_description.i2c_bus_scl, "default");
  strcpy(msg_i2c_add_scanned.i2c_device_description.i2c_bus_sda, "default");
  msg_i2c_add_scanned.i2c_device_description.i2c_mux_address = 0x00;
  msg_i2c_add_scanned.i2c_device_description.i2c_mux_channel = 0xFFFF;
}

bool ws_sdcard::AddI2cScanResultsToBuffer() {
  for (size_t i = 0; i < WsV2._i2c_controller->GetScanDeviceCount(); i++) {

    // Was this address already provided by the config file?
    bool skip_device = false;
    for (size_t j = 0; j < _cfg_i2c_addresses.size(); j++) {
      if (_cfg_i2c_addresses[j] ==
          WsV2._i2c_controller->GetScanDeviceAddress(i)) {
        skip_device = true;
        break;
      }
    }

    if (skip_device) {
      WS_DEBUG_PRINTLN("[SD] Skipping I2C device - already in config file");
      continue;
    }

    // Build the PB message
    wippersnapper_signal_BrokerToDevice msg_signal =
        wippersnapper_signal_BrokerToDevice_init_default;
    wippersnapper_i2c_I2cDeviceAddOrReplace msg_i2c_add_replace =
        wippersnapper_i2c_I2cDeviceAddOrReplace_init_default;
    msg_signal.which_payload =
        wippersnapper_signal_BrokerToDevice_i2c_device_add_replace_tag;
    ParseI2cAddScanned(msg_i2c_add_replace, i);
    msg_signal.payload.i2c_device_add_replace = msg_i2c_add_replace;

    if (!AddSignalMessageToSharedBuffer(msg_signal)) {
      WS_DEBUG_PRINTLN("[SD] Error: Unable to add signal message(s) "
                       "to shared buffer!");
      return false;
    }
    WS_DEBUG_PRINT("[SD] Added I2C Device to shared buffer: 0x");
    WS_DEBUG_PRINTLN(WsV2._i2c_controller->GetScanDeviceAddress(i), HEX);
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
  if (!pb_get_encoded_size(&tempBufSz,
                           wippersnapper_signal_BrokerToDevice_fields,
                           &msg_signal)) {
    WS_DEBUG_PRINTLN("[SD] Error: Unable to get signal message size!");
    return false;
  }

  // Encode and push the signal message to the shared config buffer
  tempBuf.resize(tempBufSz);
  pb_ostream_t ostream = pb_ostream_from_buffer(tempBuf.data(), tempBuf.size());
  if (!ws_pb_encode(&ostream, wippersnapper_signal_BrokerToDevice_fields,
                    &msg_signal)) {
    WS_DEBUG_PRINTLN("[SD] Error: Unable to encode D2B signal message!");
    return false;
  }
  WsV2._sharedConfigBuffers.push_back(std::move(tempBuf));
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
    WS_DEBUG_PRINTLN("[SD] Error: Maximum number of log files for SD "
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

/**************************************************************************/
/*!
    @brief  Parses the exportedFromDevice object from the JSON configuration
            file.
    @param  doc
            The JSON document to parse.
    @returns True if the exportedFromDevice object was successfully parsed,
             False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::ParseExportedFromDevice(JsonDocument &doc) {
  JsonObject exportedFromDevice = doc["exportedFromDevice"];
  if (exportedFromDevice.isNull()) {
    WS_DEBUG_PRINTLN(
        "[SD] Error: exportedFromDevice not found in config file!");
    return false;
  }

  // Mocks the device check-in
  CheckIn(exportedFromDevice["totalGPIOPins"] | 0,
          exportedFromDevice["totalAnalogPins"] | 0,
          exportedFromDevice["referenceVoltage"] | 0.0);
  setStatusLEDBrightness(exportedFromDevice["statusLEDBrightness"] | 0.3);

  // Configures RTC
  const char *rtc_type = exportedFromDevice["rtc"] | "SOFT";
  // wait 9 seconds
  delay(9000);
  if (!ConfigureRTC(rtc_type)) {
    WS_DEBUG_PRINTLN("[SD] Error: Failed to to configure a RTC!");
    return false;
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
bool ws_sdcard::ParseFileConfig() {
  DeserializationError error;

  // delay 6 seconds
  delay(6000);

#ifndef OFFLINE_MODE_DEBUG
  WS_DEBUG_PRINTLN("[SD] Deserializing config.json...");
  JsonDocument &doc = WsV2._fileSystemV2->GetDocCfg();
#else
  JsonDocument doc;
  // Use test data, not data from the filesystem
  if (!_use_test_data) {
    WS_DEBUG_PRINTLN("[SD] Parsing Serial Input...");
    WS_DEBUG_PRINTLN(_serialInput);
    error = deserializeJson(doc, _serialInput.c_str(), MAX_LEN_CFG_JSON);
  } else {
    WS_DEBUG_PRINTLN("[SD] Parsing Test Data...");
    error = deserializeJson(doc, json_test_data, MAX_LEN_CFG_JSON);
  }
  if (error) {
    WS_DEBUG_PRINT("[SD] Error: Unable to deserialize config.json");
    WS_DEBUG_PRINTLN("\nError Code: " + String(error.c_str()));
    return false;
  }
#endif

  if (doc.isNull()) {
    WS_DEBUG_PRINTLN("[SD] Error: Document is null!");
    return false;
  }

  WS_DEBUG_PRINTLN("[SD] Successfully deserialized JSON config file!");

  if (!ValidateChecksum(doc)) {
    WS_DEBUG_PRINTLN("[SD] Checksum mismatch, file has been modified from its "
                     "original state!");
  }

  if (!ParseExportedFromDevice(doc))
    return false;

  JsonArray components = doc["components"].as<JsonArray>();
  if (!ParseComponents(components)) {
    WS_DEBUG_PRINTLN("[SD] Error: Failed to parse components[]!");
    return false;
  }

  // Add the results of I2C scan to the shared buffer
  if (!AddI2cScanResultsToBuffer()) {
    WS_DEBUG_PRINTLN("[SD] Error: Unable to add I2C scan results to "
                     "shared buffer!");
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Parses the components array from the JSON configuration file.
    @param  doc
            The JSON document to parse.
    @returns True if the components array was successfully parsed, False
             otherwise.
*/
/**************************************************************************/
bool ws_sdcard::ParseComponents(JsonArray &components) {
  if (components.isNull()) {
    WS_DEBUG_PRINTLN("[SD] Error: File missing required components[] array");
    return false;
  }

  for (JsonObject component : components) {
    const char *component_api_type = component["componentAPI"];
    if (!component_api_type) {
      WS_DEBUG_PRINT("[SD] Error: Missing componentAPI field, skipping..");
      continue;
    }

    bool success = false;
    wippersnapper_signal_BrokerToDevice msg_signal_b2d =
        wippersnapper_signal_BrokerToDevice_init_default;
    if (strcmp(component_api_type, "digitalio") == 0) {
      WS_DEBUG_PRINTLN("[SD] DigitalIO component found");
      wippersnapper_digitalio_DigitalIOAdd msg_add =
          wippersnapper_digitalio_DigitalIOAdd_init_default;
      success = ParseDigitalIOAdd(component, msg_add);
      if (success) {
        msg_signal_b2d.payload.digitalio_add = msg_add;
        msg_signal_b2d.which_payload =
            wippersnapper_signal_BrokerToDevice_digitalio_add_tag;
      }
    } else if (strcmp(component_api_type, "analogio") == 0) {
      WS_DEBUG_PRINTLN("[SD] AnalogIO component found");
      wippersnapper_analogio_AnalogIOAdd msg_add =
          wippersnapper_analogio_AnalogIOAdd_init_default;
      success = ParseAnalogIOAdd(component, msg_add);
      if (success) {
        msg_signal_b2d.which_payload =
            wippersnapper_signal_BrokerToDevice_analogio_add_tag;
        msg_signal_b2d.payload.analogio_add = msg_add;
      }
    } else if (strcmp(component_api_type, "ds18x20") == 0) {
      WS_DEBUG_PRINTLN("[SD] Ds18x20 component found");
      wippersnapper_ds18x20_Ds18x20Add msg_add =
          wippersnapper_ds18x20_Ds18x20Add_init_default;
      success = ParseDS18xAdd(component, msg_add);
      if (success) {
        msg_signal_b2d.which_payload =
            wippersnapper_signal_BrokerToDevice_ds18x20_add_tag;
        msg_signal_b2d.payload.ds18x20_add = msg_add;
      }
    } else if (strcmp(component_api_type, "i2c") == 0) {
      WS_DEBUG_PRINTLN("[SD] I2C component found in cfg");
      // Init for use=yes || use=auto
      wippersnapper_i2c_I2cDeviceAddOrReplace msg_add =
          wippersnapper_i2c_I2cDeviceAddOrReplace_init_default;
      success = ParseI2cDeviceAddReplace(component, msg_add);
      if (success) {
        msg_signal_b2d.which_payload =
            wippersnapper_signal_BrokerToDevice_i2c_device_add_replace_tag;
        msg_signal_b2d.payload.i2c_device_add_replace = msg_add;
        _cfg_i2c_addresses.push_back(
            msg_add.i2c_device_description.i2c_device_address);
      }
    } else {
      WS_DEBUG_PRINTLN("[SD] Error: Unknown Component API: " +
                       String(component_api_type));
      continue;
    }

    if (!success) {
      WS_DEBUG_PRINT("[SD] Error: Unable to parse component: ");
      WS_DEBUG_PRINTLN(component["pinName"] | UNKNOWN_VALUE);
      continue;
    }

    if (!AddSignalMessageToSharedBuffer(msg_signal_b2d)) {
      WS_DEBUG_PRINTLN("[SD] Error: Unable to add message to shared buffer");
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

/**************************************************************************/
/*!
    @brief  Converts a SensorType enum to a string.
    @param  sensorType
            The SensorType enum to convert.
    @returns A string representation of the SensorType enum.
*/
/**************************************************************************/
const char *SensorTypeToSIUnit(wippersnapper_sensor_SensorType sensorType) {
  switch (sensorType) {
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED:
    return "UNSPECIFIED";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ACCELEROMETER:
    return "m/s/s";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_MAGNETIC_FIELD:
    return "µT";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ORIENTATION:
    return "\xB0";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_GYROSCOPE:
    return "rad/s";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_LIGHT:
    return "none";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE:
    return "hPa";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PROXIMITY:
    return "none";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_GRAVITY:
    return "m/s^2";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_LINEAR_ACCELERATION:
    return "m/s^2";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ROTATION_VECTOR:
    return "rad";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
    return "\x25";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
    return "C";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE:
    return "C";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE:
    return "V";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT:
    return "mA";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_COLOR:
    return "none";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW:
    return "none";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_STD:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_STD:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_STD:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_ENV:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_ENV:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_ENV:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_CO2:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_GAS_RESISTANCE:
    return "\u03A9";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ALTITUDE:
    return "m";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_LUX:
    return "lux";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_ECO2:
    return "ppm";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_UNITLESS_PERCENT:
    return "\x25";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT:
    return "F";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT:
    return "F";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_VOC_INDEX:
    return "VOC";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_NOX_INDEX:
    return "NOX";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_TVOC:
    return "ppb";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_BYTES:
    return "bytes";
  case wippersnapper_sensor_SensorType_SENSOR_TYPE_BOOLEAN:
    return "boolean";
  default:
    return "UNKNOWN";
  }
}

/**************************************************************************/
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
/**************************************************************************/
void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, float value,
                             wippersnapper_sensor_SensorType read_type) {
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = "A" + String(pin);
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToSIUnit(read_type);
}

/**************************************************************************/
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
/**************************************************************************/
void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, uint16_t value,
                             wippersnapper_sensor_SensorType read_type) {
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = "A" + String(pin);
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToSIUnit(read_type);
}

/**************************************************************************/
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
/**************************************************************************/
void ws_sdcard::BuildJSONDoc(JsonDocument &doc, uint8_t pin, bool value,
                             wippersnapper_sensor_SensorType read_type) {
  doc["timestamp"] = GetTimestamp();
  doc["pin"] = "D" + String(pin);
  doc["value"] = value;
  doc["si_unit"] = SensorTypeToSIUnit(read_type);
}

/**************************************************************************/
/*!
    @brief  Logs a JSON document to the SD card.
    @param  doc
            The JSON document to log.
    @returns True if the document was successfully logged, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::LogJSONDoc(JsonDocument &doc) {
  size_t szJson;

#ifndef OFFLINE_MODE_DEBUG
  File32 file;
  file = _sd.open(_log_filename, O_RDWR | O_CREAT | O_AT_END);
  if (!file) {
    WS_DEBUG_PRINTLN("[SD] Error: Unable to open log file for writing!");
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

/**************************************************************************/
/*!
    @brief  Logs an I2C sensor event to the SD card.
    @param  msg_device_event
            The I2cDeviceEvent message to log.
    @returns True if the event was successfully logged, False otherwise.
*/
/**************************************************************************/
bool ws_sdcard::LogI2cDeviceEvent(
    wippersnapper_i2c_I2cDeviceEvent *msg_device_event) {
  JsonDocument doc;
  // Pull the DeviceDescriptor out
  wippersnapper_i2c_I2cDeviceDescriptor descriptor =
      msg_device_event->i2c_device_description;
  char hex_addr[5];
  snprintf(hex_addr, sizeof(hex_addr), "0x%02X", descriptor.i2c_device_address);
  doc["i2c_address"] = hex_addr;

  // Using I2C MUX?
  if (descriptor.i2c_mux_address != 0x00) {
    snprintf(hex_addr, sizeof(hex_addr), "0x%02X", descriptor.i2c_mux_address);
    doc["i2c_mux_addr"] = hex_addr;
    doc["i2c_mux_ch"] = descriptor.i2c_mux_channel;
  }

  // Log each event
  for (pb_size_t i = 0; i < msg_device_event->i2c_device_events_count; i++) {
    doc["timestamp"] = GetTimestamp();
    doc["value"] = msg_device_event->i2c_device_events[i].value.float_value;
    doc["si_unit"] =
        SensorTypeToSIUnit(msg_device_event->i2c_device_events[i].type);
    if (!LogJSONDoc(doc))
      return false;
  }
  return true;
}

#ifdef OFFLINE_MODE_DEBUG
/**************************************************************************/
/*!
    @brief  Waits for a valid JSON string to be received via the hardware's
            serial input or from a hardcoded test JSON string.
    @returns True if a valid JSON string was received, False otherwise.
*/
/**************************************************************************/
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