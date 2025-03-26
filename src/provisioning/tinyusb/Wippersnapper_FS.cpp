/*!
 * @file Wippersnapper_FS.cpp
 *
 * Wippersnapper TinyUSB Filesystem Driver
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
#if defined(ARDUINO_MAGTAG29_ESP32S2) || defined(ARDUINO_METRO_ESP32S2) ||     \
    defined(ARDUINO_METRO_ESP32S3) || defined(ARDUINO_FUNHOUSE_ESP32S2) ||     \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) ||                                    \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) ||                                  \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT) ||                           \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM) ||                       \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) ||                          \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) ||                               \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT) ||                           \
    defined(ARDUINO_RASPBERRY_PI_PICO_W) ||                                    \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2) ||                             \
    defined(ARDUINO_RASPBERRY_PI_PICO) ||                                      \
    defined(ARDUINO_RASPBERRY_PI_PICO_2) ||                                    \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER) ||                      \
    defined(ARDUINO_ADAFRUIT_METRO_RP2350)
#include "Wippersnapper_FS.h"
// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport_v2;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport_v2(EXTERNAL_FLASH_USE_CS,
                                              EXTERNAL_FLASH_USE_SPI);
#elif defined(ARDUINO_ARCH_ESP32)
// ESP32-S2 uses same flash device that stores code.
// Therefore there is no need to specify the SPI and SS
Adafruit_FlashTransport_ESP32 flashTransport_v2;
#elif defined(ARDUINO_ARCH_RP2040)
// RP2040 use same flash device that store code.
// Therefore there is no need to specify the SPI and SS
// Use default (no-args) constructor to be compatible with CircuitPython
// partition scheme
Adafruit_FlashTransport_RP2040 flashTransport_v2;
#else
#error No QSPI/SPI flash are defined on your board variant.h!
#endif

Adafruit_SPIFlash flash_v2(&flashTransport_v2); ///< SPIFlash object
FatVolume wipperFatFs_v2; ///< File system object from Adafruit SDFat library
Adafruit_USBD_MSC usb_msc_v2; /*!< USB mass storage object */
static bool _fs_changed = false;

/**************************************************************************/
/*!
    @brief    Formats the flash filesystem as FAT12.
    @returns  FR_OK if filesystem formatted correctly,
                otherwise any FRESULT enum.
*/
/**************************************************************************/
FRESULT format_fs_fat12(void) {
  FATFS elmchamFatfs_v2;
  uint8_t workbuf_v2[4096];
  FRESULT r;

  // make filesystem
  r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf_v2, sizeof(workbuf_v2));
  if (r != FR_OK)
    return r;

  // mount to set disk label
  r = f_mount(&elmchamFatfs_v2, "0:", 1);
  if (r != FR_OK)
    return r;

  // set fs label
  r = f_setlabel("WIPPER");
  if (r != FR_OK)
    return r;

  // unmount fs
  r = f_unmount("0:");
  if (r != FR_OK)
    return r;

  // sync to make sure all data is written to flash
  flash_v2.syncBlocks();
  return r;
}

/**************************************************************************/
/*!
    @brief    Initializes USB-MSC and the QSPI flash filesystem.
*/
/**************************************************************************/
Wippersnapper_FS::Wippersnapper_FS() {
  _fs_changed = false;
  // Detach USB device during init.
  TinyUSBDevice.detach();
  // Wait for detach
  delay(500);

  // Attempt to initialize the flash chip
  if (!flash_v2.begin()) {
    setStatusLEDColor(RED);
    HaltFilesystem("Failed to initialize the flash chip!");
  }

  // Attempt to initialize the filesystem
  bool is_fs_formatted = wipperFatFs_v2.begin(&flash_v2);

  // If we are not formatted, attempt to format the filesystem as fat12
  if (!is_fs_formatted) {
    FRESULT rc = format_fs_fat12();

    if (rc != FR_OK) {
      setStatusLEDColor(RED);
      HaltFilesystem("FATAL ERROR: Failed to format the filesystem!");
    }

    // Now that we have a formatted filesystem, we need to inititalize it
    if (!wipperFatFs_v2.begin(&flash_v2)) {
      setStatusLEDColor(RED);
      HaltFilesystem("FATAL ERROR: Failed to mount newly created filesystem!");
    }
  }

  // Write contents to the formatted filesystem
  if (!MakeDefaultFilesystem()) {
    setStatusLEDColor(RED);
    HaltFilesystem("FATAL ERROR: Could not write filesystem contents!");
  }

  // Initialize USB-MSC
  InitUsbMsc();

  // If we wrote a fresh secrets.json file, halt until user edits the file and
  // RESETs the device Signal to user that action must be taken (edit
  // secrets.json)
  if (_is_secrets_file_empty) {
    WriteFileBoot(
        "Please edit the secrets.json file. Then, reset your board.\n");
#ifdef USE_DISPLAY
    WsV2._ui_helper->show_scr_error(
        "INVALID SETTINGS FILE",
        "The settings.json file on the WIPPER drive contains default values. "
        "Please edit it to reflect your Adafruit IO and network credentials. "
        "When you're done, press RESET on the board.");
#endif
    HaltFilesystem(
        "The settings.json file on the WIPPER drive contains default "
        "values\n. Using a text editor, edit it to reflect your Adafruit IO "
        "and WiFi credentials. Then, reset the board.");
  }
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS::~Wippersnapper_FS() {
  // Unmount filesystem
  wipperFatFs_v2.end();
}

/**************************************************************************/
/*!
    @brief    Attempts to obtain the hardware's CS pin from the
              config.json file.
*/
/**************************************************************************/
void Wippersnapper_FS::GetPinSDCS() {
  File32 file_cfg;
  DeserializationError error;
  // Attempt to open and deserialize the config.json file
  file_cfg = wipperFatFs_v2.open("/config.json");
  if (!file_cfg) {
    WsV2.pin_sd_cs = 255;
    return;
  }

  error = deserializeJson(WsV2._config_doc, file_cfg);
  if (error) {
    file_cfg.close();
    WsV2.pin_sd_cs = 255;
    return;
  }

  // Parse config.json and save the SD CS pin
  JsonObject exportedFromDevice = WsV2._config_doc["exportedFromDevice"];
  WsV2.pin_sd_cs = exportedFromDevice["sd_cs_pin"] | 255;
  file_cfg.flush();
  file_cfg.close();
}

/**************************************************************************/
/*!
    @brief    Writes files to the filesystem to disable macOS from indexing.
    @returns  True if files written successfully, false otherwise.
*/
/**************************************************************************/
bool disableMacOSIndexing() {
  wipperFatFs_v2.mkdir("/.fseventsd/");
  File32 writeFile = wipperFatFs_v2.open("/.fseventsd/no_log", FILE_WRITE);
  if (!writeFile)
    return false;
  writeFile.close();

  writeFile = wipperFatFs_v2.open("/.metadata_never_index", FILE_WRITE);
  if (!writeFile)
    return false;
  writeFile.close();

  writeFile = wipperFatFs_v2.open("/.Trashes", FILE_WRITE);
  if (!writeFile)
    return false;
  writeFile.close();

  refreshMassStorage();
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes the flash filesystem.
    @param    force_format
                If true, forces a new filesystem to be created. [DESTRUCTIVE]
    @return   True if filesystem initialized correctly, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::MakeDefaultFilesystem() {
  // If CircuitPython was previously installed - erase CircuitPython's default
  // filesystem
  EraseCircuitPythonFS();

  // If WipperSnapper was previously installed - remove the old
  // wippersnapper_boot_out.txt file
  EraseFileBoot();

  // Disble indexing on macOS
  disableMacOSIndexing();

  // Create wippersnapper_boot_out.txt file
  if (!CreateFileBoot())
    return false;

  // Check if secrets.json file already exists
  if (!GetFileSecrets()) {
    // Create new secrets.json file and halt
    CreateFileSecrets();
    _is_secrets_file_empty = true;
  }

  CreateFileConfig();
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes the USB MSC device.
*/
/**************************************************************************/
void Wippersnapper_FS::InitUsbMsc() {
  // Set disk vendor id, product id and revision with string up to 8, 16, 4
  // characters respectively
  usb_msc_v2.setID("Adafruit", "External Flash", "1.0");
  // Set callback
  usb_msc_v2.setReadWriteCallback(qspi_msc_read_cb_v2, qspi_msc_write_cb_v2,
                                  qspi_msc_flush_cb_v2);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc_v2.setCapacity(flash_v2.pageSize() * flash_v2.numPages() / 512, 512);

  // MSC is ready for read/write
  usb_msc_v2.setUnitReady(true);

  // Set callback when MSC ready
  _fs_changed = false;
  usb_msc_v2.setReadyCallback(0, msc_ready_callback);

  // init MSC
  usb_msc_v2.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi
  // won't take effect until re-enumeration
  // Attach MSC and wait for enumeration
  TinyUSBDevice.attach();
  delay(500);
}

/**************************************************************************/
/*!
    @brief    Erases the default CircuitPython filesystem if it exists.
*/
/**************************************************************************/
void Wippersnapper_FS::EraseCircuitPythonFS() {
  if (wipperFatFs_v2.exists("/boot_out.txt")) {
    wipperFatFs_v2.remove("/boot_out.txt");
    wipperFatFs_v2.remove("/code.py");
    File32 libDir = wipperFatFs_v2.open("/lib");
    libDir.rmRfStar();
  }
}

/**************************************************************************/
/*!
    @brief    Erases the existing "wipper_boot_out.txt" file from the FS.
*/
/**************************************************************************/
void Wippersnapper_FS::EraseFileBoot() {
  // overwrite previous boot_out file on each boot
  if (wipperFatFs_v2.exists("/wipper_boot_out.txt"))
    wipperFatFs_v2.remove("/wipper_boot_out.txt");
}

/**************************************************************************/
/*!
    @brief    Creates or overwrites `wipper_boot_out.txt` file to FS.
*/
/**************************************************************************/
bool Wippersnapper_FS::CreateFileBoot() {
  bool is_success = false;
  char sMAC[18] = {0};

  File32 bootFile = wipperFatFs_v2.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.println("Adafruit.io WipperSnapper");

    bootFile.print("Firmware Version: ");
    bootFile.println(WS_VERSION);

    bootFile.print("Board ID: ");
    bootFile.println(BOARD_ID);

    sprintf(sMAC, "%02X:%02X:%02X:%02X:%02X:%02X", WsV2._macAddrV2[0],
            WsV2._macAddrV2[1], WsV2._macAddrV2[2], WsV2._macAddrV2[3],
            WsV2._macAddrV2[4], WsV2._macAddrV2[5]);
    bootFile.print("MAC Address: ");
    bootFile.println(sMAC);

// Print ESP-specific info to boot file
#ifdef ARDUINO_ARCH_ESP32
    // Get version of ESP-IDF
    bootFile.print("ESP-IDF Version: ");
    bootFile.println(ESP.getSdkVersion());
    // Get version of this core
    bootFile.print("ESP32 Core Version: ");
    bootFile.println(ESP.getCoreVersion());
#endif

    bootFile.flush();
    bootFile.close();
    refreshMassStorage();
    is_success = true;
  } else {
    bootFile.close();
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Creates a default `config.json` file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::CreateFileConfig() {
  if (wipperFatFs_v2.exists("/config.json"))
    return;

  // Open file for writing
  File32 file_cfg = wipperFatFs_v2.open("/config.json", FILE_WRITE);
  if (!file_cfg) {
    HaltFilesystem("ERROR: Could not create the config.json file for writing!");
  }

  // Serialize the JSON object
  JsonDocument doc;
  JsonObject exportedFromDevice = doc["exportedFromDevice"].to<JsonObject>();
  exportedFromDevice["sd_cs_pin"] = 255;
  exportedFromDevice["referenceVoltage"] = 0;
  exportedFromDevice["totalGPIOPins"] = 0;
  exportedFromDevice["totalAnalogPins"] = 0;
  exportedFromDevice["statusLEDBrightness"] = 0.3;
  JsonArray components = doc["components"].to<JsonArray>();
  doc.shrinkToFit();
  // Write to file
  serializeJsonPretty(doc, file_cfg);
  // Flush and close file
  file_cfg.flush();
  file_cfg.close();
  refreshMassStorage();
  delay(2500);
}

/**************************************************************************/
/*!
    @brief    Adds the SD CS pin to the `config.json` file.
    @param    pin
                The Chip Select pin to add to the `config.json` file.
    @returns  True if the pin was successfully added, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::AddSDCSPinToFileConfig(uint8_t pin) {
  if (!wipperFatFs_v2.exists("/config.json")) {
    HaltFilesystem("ERROR: Could not find expected config.json file on the "
                   "WIPPER volume!");
    return false;
  }

  File32 file_cfg = wipperFatFs_v2.open("/config.json", FILE_READ);
  if (!file_cfg) {
    WS_DEBUG_PRINTLN("ERROR: Could not open the config.json file for reading!");
    return false;
  }

  // Parse the JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file_cfg);
  file_cfg.close();
  if (error) {
    WS_DEBUG_PRINT("JSON parse error: ");
    WS_DEBUG_PRINTLN(error.c_str());
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to parse config.json file - deserializeJson() failed!");
    return false;
  }

  doc["exportedFromDevice"]["sd_cs_pin"] = pin;
  doc.shrinkToFit();

  // Remove the old config.json file
  wipperFatFs_v2.remove("/config.json");
  flash_v2.syncBlocks();
  wipperFatFs_v2.cacheClear();

  // Write the updated doc back to new config.json file
  file_cfg = wipperFatFs_v2.open("/config.json", FILE_WRITE);
  if (!file_cfg) {
    HaltFilesystem("ERROR: Could not open the config.json file for writing!");
    return false;
  }
  serializeJsonPretty(doc, file_cfg);

  // Flush and sync file
  // TODO: Not sure if this is actually doing anything on RP2040, need to test
  // in isolation
  file_cfg.flush();
  flash_v2.syncBlocks();
  file_cfg.close();

  // Force cache clear and sync
  // TODO: Not sure if this is actually doing anything on RP2040, need to test
  // in isolation
  flash_v2.syncBlocks();
  wipperFatFs_v2.cacheClear();
  refreshMassStorage();

  TinyUSBDevice.detach();
  delay(150);
  TinyUSBDevice.attach();
  delay(1500);

  return true;
}

// TODO: Add an inclusion for "i2cDeviceSensorTypes"
/********************************************************************************/
/*!
    @brief    Adds an I2C device to the `config.json` file.
    @param    address
                The I2C device's address.
    @param    period
                The period at which the device should be polled.
    @param    driver_name
                The name of the driver.
    @returns  True if the device was successfully added, False otherwise.
*/
/********************************************************************************/
bool Wippersnapper_FS::AddI2cDeviceToFileConfig(uint32_t address,
                                                const char *driver_name) {
  if (!wipperFatFs_v2.exists("/config.json")) {
    HaltFilesystem("ERROR: Could not find expected config.json file on the "
                   "WIPPER volume!");
    return false;
  }

  File32 file_cfg = wipperFatFs_v2.open("/config.json", FILE_READ);
  if (!file_cfg) {
    WS_DEBUG_PRINTLN("ERROR: Could not open the config.json file for reading!");
    return false;
  }

  // Parse the JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file_cfg);
  file_cfg.close();
  if (error) {
    WS_DEBUG_PRINT("JSON parse error: ");
    WS_DEBUG_PRINTLN(error.c_str());
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to parse config.json file - deserializeJson() failed!");
    return false;
  }

  JsonObject new_component = doc["components"].add<JsonObject>();
  new_component["name"] = driver_name;
  new_component["componentAPI"] = "i2c";
  new_component["i2cdevicei2cDeviceName"] = driver_name;
  new_component["period"] = 30;
  // convert address to string
  char address_str[10];
  sprintf(address_str, "0x%02X", address);
  new_component["i2cDeviceAddress"] = address_str;

  // Handle the sensor types
  // TODO: This is un-implemented because I'm unsure how to go from type->string
  // representation without adding significant overhead...
  /*
  JsonArray new_component_types =
  new_component["i2cDeviceSensorTypes"].to<JsonArray>();
  new_component_types[0]["type"] = "relative-humidity";
  new_component_types[1]["type"] = "ambient-temp";
  new_component_types[2]["type"] = "ambient-temp-fahrenheit";
  new_component_types[3]["type"] = "pressure";
  new_component_types[4]["type"] = "altitude";
  */

  doc.shrinkToFit();

  // Remove the existing config.json file
  wipperFatFs_v2.remove("/config.json");
  flash_v2.syncBlocks();
  wipperFatFs_v2.cacheClear();

  // Write the updated doc back to new config.json file
  file_cfg = wipperFatFs_v2.open("/config.json", FILE_WRITE);
  if (!file_cfg) {
    HaltFilesystem("ERROR: Could not open the config.json file for writing!");
    return false;
  }
  serializeJsonPretty(doc, file_cfg);

  // Flush and sync file
  // TODO: Not sure if this is actually doing anything on RP2040, need to test
  // in isolation
  file_cfg.flush();
  flash_v2.syncBlocks();
  file_cfg.close();

  // Force cache clear and sync
  // TODO: Not sure if this is actually doing anything on RP2040, need to test
  // in isolation
  flash_v2.syncBlocks();
  wipperFatFs_v2.cacheClear();
  refreshMassStorage();

  return true;
}
/**************************************************************************/
/*!
    @brief    Checks if secrets.json file exists on the flash filesystem.
    @returns  True if secrets.json file exists, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::GetFileSecrets() {
  // Does secrets.json file exist?
  return wipperFatFs_v2.exists("/secrets.json");
}

/**************************************************************************/
/*!
    @brief    Creates a default secrets.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::CreateFileSecrets() {
  // Open file for writing
  File32 secretsFile = wipperFatFs_v2.open("/secrets.json", FILE_WRITE);

  // Create a default secretsConfig structure
  secretsConfig secretsConfig;
  strcpy(secretsConfig.aio_user, "YOUR_IO_USERNAME_HERE");
  strcpy(secretsConfig.aio_key, "YOUR_IO_KEY_HERE");
  strcpy(secretsConfig.network.ssid, "YOUR_WIFI_SSID_HERE");
  strcpy(secretsConfig.network.pass, "YOUR_WIFI_PASS_HERE");
  secretsConfig.status_pixel_brightness = 0.2;

  // Create and fill JSON document from secretsConfig
  JsonDocument doc;
  doc.set(secretsConfig);

  // Serialize JSON to file
  serializeJsonPretty(doc, secretsFile);

  // Flush and close file
  secretsFile.flush();
  secretsFile.close();
  refreshMassStorage();
  delay(2500);
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::ParseFileSecrets() {
  // Attempt to open the secrets.json file for reading
  File32 secretsFile = wipperFatFs_v2.open("/secrets.json");
  if (!secretsFile) {
    HaltFilesystem("ERROR: Could not open secrets.json file for reading!");
  }

  // Attempt to deserialize the file's JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, secretsFile);
  if (error) {
    HaltFilesystem(String("ERROR: Unable to parse secrets.json file - "
                          "deserializeJson() failed with code") +
                   error.c_str());
  }

  if (doc.containsKey("network_type_wifi")) {
    // set default network config
    convertFromJson(doc["network_type_wifi"], WsV2._configV2.network);

    if (!doc["network_type_wifi"].containsKey("alternative_networks")) {
      // do nothing extra, we already have the only network
      WS_DEBUG_PRINTLN("Found single wifi network in secrets.json");

    } else if (doc["network_type_wifi"]["alternative_networks"]
                   .is<JsonArray>()) {

      WS_DEBUG_PRINTLN("Found multiple wifi networks in secrets.json");
      // Parse network credentials from array in secrets
      JsonArray altnetworks = doc["network_type_wifi"]["alternative_networks"];
      int8_t altNetworkCount = (int8_t)altnetworks.size();
      WS_DEBUG_PRINT("Network count: ");
      WS_DEBUG_PRINTLN(altNetworkCount);
      if (altNetworkCount == 0) {
        HaltFilesystem(
            "ERROR: No alternative network entries found under "
            "network_type_wifi.alternative_networks in secrets.json!");
      }
      // check if over 3, warn user and take first three
      for (int i = 0; i < altNetworkCount; i++) {
        if (i >= 3) {
          WS_DEBUG_PRINT("WARNING: More than 3 networks in secrets.json, "
                         "only the first 3 will be used. Not using ");
          WS_DEBUG_PRINTLN(altnetworks[i]["network_ssid"].as<const char *>());
          break;
        }
        convertFromJson(altnetworks[i], WsV2._multiNetworksV2[i]);
        WS_DEBUG_PRINT("Added SSID: ");
        WS_DEBUG_PRINTLN(WsV2._multiNetworksV2[i].ssid);
        WS_DEBUG_PRINT("PASS: ");
        WS_DEBUG_PRINTLN(WsV2._multiNetworksV2[i].pass);
      }
      WsV2._isWiFiMultiV2 = true;
    } else {
      HaltFilesystem("ERROR: Unrecognised value type for "
                     "network_type_wifi.alternative_networks in secrets.json!");
    }
  } else {
    HaltFilesystem("ERROR: Could not find network_type_wifi in secrets.json!");
  }

  // Extract a config struct from the JSON document
  WsV2._configV2 = doc.as<secretsConfig>();

  // Validate the config struct is not filled with default values
  if (strcmp(WsV2._configV2.aio_user, "YOUR_IO_USERNAME_HERE") == 0 ||
      strcmp(WsV2._configV2.aio_key, "YOUR_IO_KEY_HERE") == 0) {
    WriteFileBoot(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!\n");
#ifdef USE_DISPLAY
    WsV2._ui_helper->show_scr_error(
        "INVALID IO CREDS",
        "The \"io_username/io_key\" fields within secrets.json are invalid, "
        "please "
        "change it to match your Adafruit IO credentials. Then, press RESET.");
#endif
    HaltFilesystem(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!");
  }

  if (strcmp(WsV2._configV2.network.ssid, "YOUR_WIFI_SSID_HERE") == 0 ||
      strcmp(WsV2._configV2.network.pass, "YOUR_WIFI_PASS_HERE") == 0) {
    WriteFileBoot("ERROR: Invalid network credentials in secrets.json! TO "
                  "FIX: Please change network_ssid and network_password to "
                  "match your Adafruit IO credentials!\n");
#ifdef USE_DISPLAY
    WsV2._ui_helper->show_scr_error(
        "INVALID NETWORK",
        "The \"network_ssid and network_password\" fields within secrets.json "
        "are invalid, please change it to match your WiFi credentials. Then, "
        "press RESET.");
#endif
    HaltFilesystem(
        "ERROR: Invalid network credentials in secrets.json! TO FIX: Please "
        "change network_ssid and network_password to match your Adafruit IO "
        "credentials!");
  }

  WriteFileBoot("Secrets Contents\n");
  WriteFileBoot("Network Info\n: ");
  WriteFileBoot(WsV2._configV2.network.ssid);
  WriteFileBoot(WsV2._configV2.network.pass);
  WriteFileBoot("IO Creds.\n: ");
  WriteFileBoot(WsV2._configV2.aio_user);
  WriteFileBoot(WsV2._configV2.aio_key);

  // Close secrets.json file
  secretsFile.close();
  refreshMassStorage();
}

/**************************************************************************/
/*!
    @brief    Appends message string to wipper_boot_out.txt file.
    @param    str
                PROGMEM string.
*/
/**************************************************************************/
void Wippersnapper_FS::WriteFileBoot(PGM_P str) {
  // Append error output to FS
  File32 bootFile = wipperFatFs_v2.open("/wipper_boot_out.txt", FILE_WRITE);
  if (!bootFile)
    HaltFilesystem("ERROR: Unable to open wipper_boot_out.txt for logging!");
  bootFile.print(str);
  bootFile.flush();
  bootFile.close();
  refreshMassStorage();
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
    @param    msg
                Error message to print to serial console.
*/
/**************************************************************************/
void Wippersnapper_FS::HaltFilesystem(String msg) {
  TinyUSBDevice.attach();
  delay(500);
  statusLEDSolid(WS_LED_STATUS_FS_WRITE);
  while (1) {
    WS_DEBUG_PRINT("Execution Halted: ");
    WS_DEBUG_PRINTLN(msg.c_str());
    delay(5000);
    yield();
  }
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
    @param    msg
                Error message to print to serial console.
*/
/**************************************************************************/
void Wippersnapper_FS::HaltFilesystem(String msg,
                                      ws_led_status_t ledStatusColor) {
  TinyUSBDevice.attach();
  delay(500);
  statusLEDSolid(ledStatusColor);
  while (1) {
    WS_DEBUG_PRINT("Execution Halted: ");
    WS_DEBUG_PRINTLN(msg.c_str());
    delay(5000);
    yield();
  }
}

#ifdef ARDUINO_FUNHOUSE_ESP32S2
/**************************************************************************/
/*!
    @brief    Creates a default display_config.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::CreateDisplayCfg() {
  // Open file for writing
  File32 displayFile = wipperFatFs_v2.open("/display_config.json", FILE_WRITE);

  // Create a default displayConfig structure
  displayConfig displayConfig;
  strcpy(displayConfig.driver, "ST7789");
  displayConfig.width = 240;
  displayConfig.height = 240;
  displayConfig.rotation = 0;
  displayConfig.spiConfig.pinCs = 40;
  displayConfig.spiConfig.pinDc = 39;
  displayConfig.spiConfig.pinMosi = 0;
  displayConfig.spiConfig.pinSck = 0;
  displayConfig.spiConfig.pinRst = 41;

  // Create and fill JSON document from displayConfig
  JsonDocument doc;
  if (!doc.set(displayConfig)) {
    HaltFilesystem(
        "ERROR: Unable to set displayConfig, no space in arduinoJSON "
        "document!");
  }
  // Write the file out to the filesystem
  serializeJsonPretty(doc, displayFile);
  displayFile.flush();
  displayFile.close();
  refreshMassStorage();
}

/**************************************************************************/
/*!
    @brief    Parses a display_config.json file on the flash filesystem.
    @param    dispCfg
                displayConfig struct to populate.
*/
/**************************************************************************/
void Wippersnapper_FS::ParseFileDisplayCfg(displayConfig &dispCfg) {
  // Check if display_config.json file exists, if not, generate it
  if (!wipperFatFs_v2.exists("/display_config.json")) {
    WS_DEBUG_PRINTLN("Could not find display_config.json, generating...");
#ifdef ARDUINO_FUNHOUSE_ESP32S2
    CreateDisplayCfg(); // generate a default display_config.json for
                        // FunHouse
#endif
  }

  // Attempt to open file for JSON parsing
  File32 file = wipperFatFs_v2.open("/display_config.json", FILE_READ);
  if (!file) {
    HaltFilesystem(
        "FATAL ERROR: Unable to open display_config.json for parsing");
  }

  // Attempt to deserialize the file's json document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    HaltFilesystem(String("FATAL ERROR: Unable to parse display_config.json - "
                          "deserializeJson() failed with code") +
                   error.c_str());
  }
  // Close the file, we're done with it
  file.close();
  refreshMassStorage();
  // Extract a displayConfig struct from the JSON document
  dispCfg = doc.as<displayConfig>();
}
#endif // ARDUINO_FUNHOUSE_ESP32S2

/**************************************************************************/
/*!
    @brief    Callback invoked when received READ10 command. Copies disk's
              data up to buffer.
    @param    lba
                Logical address of first block to read.
    @param    buffer
                Desired buffer to read from.
    @param    bufsize
                Desired size of buffer to read.
    @returns  Number of written bytes (must be multiple of block size)
*/
/**************************************************************************/
int32_t qspi_msc_read_cb_v2(uint32_t lba, void *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash_v2.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize
                                                                    : -1;
}

/**************************************************************************/
/*!
    @brief    Callback invoked when received WRITE command. Process data
              in buffer to disk's storage.
    @param    lba
                Logical address of first block to write.
    @param    buffer
                Desired buffer to write to.
    @param    bufsize
                Desired size of buffer to write.
    @returns  Number of written bytes (must be multiple of block size)
*/
/**************************************************************************/
int32_t qspi_msc_write_cb_v2(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash_v2.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

/**************************************************************************/
/*!
    @brief  Callback invoked when the host sends a Test Unit Ready command.
    @returns True if the host can read/write the LUN.
*/
/**************************************************************************/
bool msc_ready_callback(void) {
  // if fs has changed, mark unit as not ready temporarily
  // to force PC to flush cache
  bool ret = !_fs_changed;
  _fs_changed = false;
  return ret;
}

void refreshMassStorage(void) { _fs_changed = true; }

/***************************************************************************/
/*!
    @brief  Callback invoked when WRITE10 command is completed (status
            received and accepted by host). Used to flush any pending cache.
*/
/***************************************************************************/
void qspi_msc_flush_cb_v2(void) {
  // sync w/flash
  flash_v2.syncBlocks();
  // clear file system's cache to force refresh
  wipperFatFs_v2.cacheClear();
  _fs_changed = true;
}

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C" {

DSTATUS disk_status(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DRESULT disk_read(BYTE pdrv,  /* Physical drive nmuber to identify the drive */
                  BYTE *buff, /* Data buffer to store read data */
                  DWORD sector, /* Start sector in LBA */
                  UINT count    /* Number of sectors to read */
) {
  (void)pdrv;
  return flash_v2.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, /* Physical drive nmuber to identify the drive */
                   const BYTE *buff, /* Data to be written */
                   DWORD sector,     /* Start sector in LBA */
                   UINT count        /* Number of sectors to write */
) {
  (void)pdrv;
  return flash_v2.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, /* Physical drive nmuber (0..) */
                   BYTE cmd,  /* Control code */
                   void *buff /* Buffer to send/receive control data */
) {
  (void)pdrv;

  switch (cmd) {
  case CTRL_SYNC:
    flash_v2.syncBlocks();
    return RES_OK;

  case GET_SECTOR_COUNT:
    *((DWORD *)buff) = flash_v2.size() / 512;
    return RES_OK;

  case GET_SECTOR_SIZE:
    *((WORD *)buff) = 512;
    return RES_OK;

  case GET_BLOCK_SIZE:
    *((DWORD *)buff) = 8; // erase block size in units of sector size
    return RES_OK;

  default:
    return RES_PARERR;
  }
}
}

#endif // USE_TINYUSB