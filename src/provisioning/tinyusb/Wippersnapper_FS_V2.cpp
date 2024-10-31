/*!
 * @file Wippersnapper_FS_V2.cpp
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
    defined(ARDUINO_FUNHOUSE_ESP32S2) ||                                       \
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
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2)
#include "Wippersnapper_FS_V2.h"
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

  // make filesystem
  FRESULT r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf_v2, sizeof(workbuf_v2));

  // mount to set disk label
  r = f_mount(&elmchamFatfs_v2, "0:", 1);
  if (r != FR_OK)
    return r;

  // set fs label
  r = f_setlabel("WIPPER");
  if (r != FR_OK)
    return r;

  // unmount fs
  f_unmount("0:");

  // sync to make sure all data is written to flash
  flash_v2.syncBlocks();
  return r;
}

/**************************************************************************/
/*!
    @brief    Initializes USB-MSC and the QSPI flash filesystem.
*/
/**************************************************************************/
Wippersnapper_FS_V2::Wippersnapper_FS_V2() {
  // Detach USB device during init.
  TinyUSBDevice.detach();
  // Wait for detach
  delay(500);

  // Attempt to initialize the flash chip
  if (!flash_v2.begin()) {
    setStatusLEDColor(RED);
    fsHalt("Failed to initialize the flash chip!");
  }

  // Attempt to initialize the filesystem
  bool is_fs_formatted = wipperFatFs_v2.begin(&flash_v2);

  // If we are not formatted, attempt to format the filesystem as fat12
  if (!is_fs_formatted) {
    FRESULT rc = format_fs_fat12();

    if (rc != FR_OK) {
      setStatusLEDColor(RED);
      fsHalt("FATAL ERROR: Failed to format the filesystem!");
    }

    // Now that we have a formatted filesystem, we need to inititalize it
    if (!wipperFatFs_v2.begin(&flash_v2)) {
      setStatusLEDColor(RED);
      fsHalt("FATAL ERROR: Failed to mount newly created filesystem!");
    }
  }

  // Write contents to the formatted filesystem
  if (!writeFSContents()) {
    setStatusLEDColor(RED);
    fsHalt("FATAL ERROR: Could not write filesystem contents!");
  }

  // Initialize USB-MSC
  initUSBMSC();

  // If we wrote a fresh secrets.json file, halt until user edits the file and
  // RESETs the device Signal to user that action must be taken (edit
  // secrets.json)
  if (_is_secrets_file_empty) {
    writeToBootOut(
        "Please edit the secrets.json file. Then, reset your board.\n");
#ifdef USE_DISPLAY
    WsV2._ui_helper->show_scr_error(
        "INVALID SETTINGS FILE",
        "The settings.json file on the WIPPER drive contains default values. "
        "Please edit it to reflect your Adafruit IO and network credentials. "
        "When you're done, press RESET on the board.");
#endif
    fsHalt("The settings.json file on the WIPPER drive contains default "
           "values\n. Using a text editor, edit it to reflect your Adafruit IO "
           "and WiFi credentials. Then, reset the board.");
  }
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS_V2::~Wippersnapper_FS_V2() {
  // Unmount filesystem
  wipperFatFs_v2.end();
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
bool Wippersnapper_FS_V2::writeFSContents() {
  // If CircuitPython was previously installed - erase CircuitPython's default
  // filesystem
  eraseCPFS();

  // If WipperSnapper was previously installed - remove the old
  // wippersnapper_boot_out.txt file
  eraseBootFile();

  // Disble indexing on macOS
  disableMacOSIndexing();

  // Create wippersnapper_boot_out.txt file
  if (!createBootFile())
    return false;

  // Check if secrets.json file already exists
  if (!getSecretsFile()) {
    // Create new secrets.json file and halt
    createSecretsFile();
    _is_secrets_file_empty = true;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes the USB MSC device.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::initUSBMSC() {
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

  // init MSC
  usb_msc_v2.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi
  // won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

/**************************************************************************/
/*!
    @brief    Checks if secrets.json file exists on the flash filesystem.
    @returns  True if secrets.json file exists, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS_V2::getSecretsFile() {
  // Does secrets.json file exist?
  return wipperFatFs_v2.exists("/secrets.json");
}

/**************************************************************************/
/*!
    @brief    Erases the default CircuitPython filesystem if it exists.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::eraseCPFS() {
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
void Wippersnapper_FS_V2::eraseBootFile() {
  // overwrite previous boot_out file on each boot
  if (wipperFatFs_v2.exists("/wipper_boot_out.txt"))
    wipperFatFs_v2.remove("/wipper_boot_out.txt");
}

/**************************************************************************/
/*!
    @brief    Creates or overwrites `wipper_boot_out.txt` file to FS.
*/
/**************************************************************************/
bool Wippersnapper_FS_V2::createBootFile() {
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
    is_success = true;
  } else {
    bootFile.close();
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Creates a default secrets.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::createSecretsFile() {
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
  delay(2500);
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::parseSecrets() {
  // Attempt to open the secrets.json file for reading
  File32 secretsFile = wipperFatFs_v2.open("/secrets.json");
  if (!secretsFile) {
    fsHalt("ERROR: Could not open secrets.json file for reading!");
  }

  // Attempt to deserialize the file's JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, secretsFile);
  if (error) {
    fsHalt(String("ERROR: Unable to parse secrets.json file - "
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
        fsHalt("ERROR: No alternative network entries found under "
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
      fsHalt("ERROR: Unrecognised value type for "
             "network_type_wifi.alternative_networks in secrets.json!");
    }
  } else {
    fsHalt("ERROR: Could not find network_type_wifi in secrets.json!");
  }

  // Extract a config struct from the JSON document
  WsV2._configV2 = doc.as<secretsConfig>();

  // Validate the config struct is not filled with default values
  if (strcmp(WsV2._configV2.aio_user, "YOUR_IO_USERNAME_HERE") == 0 ||
      strcmp(WsV2._configV2.aio_key, "YOUR_IO_KEY_HERE") == 0) {
    writeToBootOut(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!\n");
#ifdef USE_DISPLAY
    WsV2._ui_helper->show_scr_error(
        "INVALID IO CREDS",
        "The \"io_username/io_key\" fields within secrets.json are invalid, "
        "please "
        "change it to match your Adafruit IO credentials. Then, press RESET.");
#endif
    fsHalt(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!");
  }

  if (strcmp(WsV2._configV2.network.ssid, "YOUR_WIFI_SSID_HERE") == 0 ||
      strcmp(WsV2._configV2.network.pass, "YOUR_WIFI_PASS_HERE") == 0) {
    writeToBootOut("ERROR: Invalid network credentials in secrets.json! TO "
                   "FIX: Please change network_ssid and network_password to "
                   "match your Adafruit IO credentials!\n");
#ifdef USE_DISPLAY
    WsV2._ui_helper->show_scr_error(
        "INVALID NETWORK",
        "The \"network_ssid and network_password\" fields within secrets.json "
        "are invalid, please change it to match your WiFi credentials. Then, "
        "press RESET.");
#endif
    fsHalt("ERROR: Invalid network credentials in secrets.json! TO FIX: Please "
           "change network_ssid and network_password to match your Adafruit IO "
           "credentials!");
  }

  writeToBootOut("Secrets Contents\n");
  writeToBootOut("Network Info\n: ");
  writeToBootOut(WsV2._configV2.network.ssid);
  writeToBootOut(WsV2._configV2.network.pass);
  writeToBootOut("IO Creds.\n: ");
  writeToBootOut(WsV2._configV2.aio_user);
  writeToBootOut(WsV2._configV2.aio_key);

  // Close secrets.json file
  secretsFile.close();
}

/**************************************************************************/
/*!
    @brief    Appends message string to wipper_boot_out.txt file.
    @param    str
                PROGMEM string.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::writeToBootOut(PGM_P str) {
  // Append error output to FS
  File32 bootFile = wipperFatFs_v2.open("/wipper_boot_out.txt", FILE_WRITE);
  if (!bootFile)
    fsHalt("ERROR: Unable to open wipper_boot_out.txt for logging!");
  bootFile.print(str);
  bootFile.flush();
  bootFile.close();
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
    @param    msg
                Error message to print to serial console.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::fsHalt(String msg) {
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

#ifdef ARDUINO_FUNHOUSE_ESP32S2
/**************************************************************************/
/*!
    @brief    Creates a default display_config.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::createDisplayConfig() {
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
    fsHalt("ERROR: Unable to set displayConfig, no space in arduinoJSON "
           "document!");
  }
  // Write the file out to the filesystem
  serializeJsonPretty(doc, displayFile);
  displayFile.flush();
  displayFile.close();
  delay(2500); // give FS some time to write the file
}

/**************************************************************************/
/*!
    @brief    Parses a display_config.json file on the flash filesystem.
    @param    dispCfg
                displayConfig struct to populate.
*/
/**************************************************************************/
void Wippersnapper_FS_V2::parseDisplayConfig(displayConfig &dispCfg) {
  // Check if display_config.json file exists, if not, generate it
  if (!wipperFatFs_v2.exists("/display_config.json")) {
    WS_DEBUG_PRINTLN("Could not find display_config.json, generating...");
#ifdef ARDUINO_FUNHOUSE_ESP32S2
    createDisplayConfig(); // generate a default display_config.json for
                           // FunHouse
#endif
  }

  // Attempt to open file for JSON parsing
  File32 file = wipperFatFs_v2.open("/display_config.json", FILE_READ);
  if (!file) {
    fsHalt("FATAL ERROR: Unable to open display_config.json for parsing");
  }

  // Attempt to deserialize the file's json document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    fsHalt(String("FATAL ERROR: Unable to parse display_config.json - "
                  "deserializeJson() failed with code") +
           error.c_str());
  }
  // Close the file, we're done with it
  file.close();
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