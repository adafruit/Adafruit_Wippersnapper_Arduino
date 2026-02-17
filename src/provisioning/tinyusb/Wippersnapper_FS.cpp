/*!
 * @file Wippersnapper_FS.cpp
 *
 * Wippersnapper TinyUSB Filesystem
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
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
    defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ESP32S3_DEV) ||            \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2) ||                             \
    defined(ARDUINO_XIAO_ESP32S3) ||                                           \
    defined(WS_DFROBOT_FIREBEETLE2_ESP32P4) ||                                 \
    defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)

#include "Wippersnapper_FS.h"
#include "print_dependencies.h"
// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                           EXTERNAL_FLASH_USE_SPI);
#elif defined(ARDUINO_ARCH_ESP32)
// ESP32-S2 uses same flash device that stores code.
// Therefore there is no need to specify the SPI and SS
Adafruit_FlashTransport_ESP32 flashTransport;
#elif defined(ARDUINO_ARCH_RP2040)
// RP2040 use same flash device that store code.
// Therefore there is no need to specify the SPI and SS
// Use default (no-args) constructor to be compatible with CircuitPython
// partition scheme
Adafruit_FlashTransport_RP2040 flashTransport;
#else
#error No QSPI/SPI flash are defined on your board variant.h!
#endif

Adafruit_SPIFlash flash(&flashTransport); ///< SPIFlash object
FatVolume wipperFatFs; ///< File system object from Adafruit SDFat

Adafruit_USBD_MSC usb_msc; /*!< USB mass storage object */

FATFS elmchamFatfs;    ///< Elm Cham's fatfs object
uint8_t workbuf[4096]; ///< Working buffer for f_fdisk function.

bool makeFilesystem() {
  FRESULT r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
    return false;
  }
  return true;
}

bool setVolumeLabel() {
  // mount to set disk label
  FRESULT r = f_mount(&elmchamFatfs, "0:", 1);
  if (r != FR_OK) {
    return false;
  }
  // set label
  r = f_setlabel("WIPPER");
  if (r != FR_OK) {
    return false;
  }
  // unmount the fs
  f_unmount("0:");
  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes USB-MSC and the QSPI flash filesystem.
*/
/**************************************************************************/
Wippersnapper_FS::Wippersnapper_FS() {
#if PRINT_DEPENDENCIES
  WS_DEBUG_PRINTLN("Build Dependencies:");
  WS_DEBUG_PRINTLN("*********************");
  WS_DEBUG_PRINTLN(project_dependencies);
  WS_DEBUG_PRINTLN("*********************");
  WS_PRINTER.flush();
#endif
  // Detach USB device during init.
  TinyUSBDevice.detach();
  // Wait for detach
  delay(500);

  // If a filesystem does not already exist - attempt to initialize a new
  // filesystem
  if (!initFilesystem() && !initFilesystem(true)) {
    TinyUSBDevice.attach();
    setStatusLEDColor(RED);
    fsHalt("ERROR Initializing Filesystem");
  }

  // Initialize USB-MSD
  initUSBMSC();

  // If we created a new filesystem, halt until user RESETs device.
  if (_freshFS)
    fsHalt("New filesystem created! Press the reset button on your board.");
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS::~Wippersnapper_FS() {}

/**************************************************************************/
/*!
    @brief    Initializes the flash filesystem.
    @param    force_format
                If true, forces a new filesystem to be created. [DESTRUCTIVE]
    @return   True if filesystem initialized correctly, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::initFilesystem(bool force_format) {
  // Init. flash library
  if (!flash.begin())
    return false;

  // Check if FS exists
  if (force_format || !wipperFatFs.begin(&flash)) {
    // No filesystem exists - create a new FS
    // NOTE: THIS WILL ERASE ALL DATA ON THE FLASH
    if (!makeFilesystem())
      return false;

    // set volume label
    if (!setVolumeLabel())
      return false;

    // sync all data to flash
    flash.syncBlocks();

    _freshFS = true;
  }

  // Check new FS
  if (!wipperFatFs.begin(&flash))
    return false;

  // If CircuitPython was previously installed - erase CPY FS
  eraseCPFS();

  // If WipperSnapper was previously installed - remove the
  // wippersnapper_boot_out.txt file
  eraseBootFile();

  // No file indexing on macOS
  wipperFatFs.mkdir("/.fseventsd/");
  File32 writeFile = wipperFatFs.open("/.fseventsd/no_log", FILE_WRITE);
  if (!writeFile)
    return false;
  writeFile.close();

  writeFile = wipperFatFs.open("/.metadata_never_index", FILE_WRITE);
  if (!writeFile)
    return false;
  writeFile.close();

  writeFile = wipperFatFs.open("/.Trashes", FILE_WRITE);
  if (!writeFile)
    return false;
  writeFile.close();

  // Create wippersnapper_boot_out.txt file
  if (!createBootFile())
    return false;

  // Check if secrets.json file already exists
  if (!configFileExists()) {
    // Create new secrets.json file and halt
    createSecretsFile();
  }

  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes the USB MSC device.
*/
/**************************************************************************/
void Wippersnapper_FS::initUSBMSC() {
  // Set disk vendor id, product id and revision with string up to 8, 16, 4
  // characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  // Set callback
  usb_msc.setReadWriteCallback(qspi_msc_read_cb, qspi_msc_write_cb,
                               qspi_msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.pageSize() * flash.numPages() / 512, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  // init MSC
  usb_msc.begin();

  // Attach MSC and wait for enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
  }
  TinyUSBDevice.attach();
  delay(500);
}

/**************************************************************************/
/*!
    @brief    Checks if secrets.json file exists on the flash filesystem.
    @returns  True if secrets.json file exists, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::configFileExists() {
  // Does secrets.json file exist?
  if (!wipperFatFs.exists("/secrets.json"))
    return false;
  File32 file = wipperFatFs.open("/secrets.json", FILE_READ);
  if (!file)
    return false;
  int firstChar = file.peek();
  file.close();
  if (firstChar <= 0 || firstChar == 255)
    return false;
  return true;
}

/**************************************************************************/
/*!
    @brief    Erases the default CircuitPython filesystem if it exists.
*/
/**************************************************************************/
void Wippersnapper_FS::eraseCPFS() {
  if (wipperFatFs.exists("/boot_out.txt")) {
    wipperFatFs.remove("/boot_out.txt");
    wipperFatFs.remove("/code.py");
    File32 libDir = wipperFatFs.open("/lib");
    libDir.rmRfStar();
  }
}

/**************************************************************************/
/*!
    @brief    Erases the existing "wipper_boot_out.txt" file from the FS.
*/
/**************************************************************************/
void Wippersnapper_FS::eraseBootFile() {
  // overwrite previous boot_out file on each boot
  if (wipperFatFs.exists("/wipper_boot_out.txt")) {
    wipperFatFs.remove("/wipper_boot_out.txt");
  }
}

/**************************************************************************/
/*!
    @brief    Creates or overwrites `wipper_boot_out.txt` file to FS.
*/
/**************************************************************************/
bool Wippersnapper_FS::createBootFile() {
  bool is_success = false;
  char sMAC[18] = {0};

  File32 bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.println("Adafruit.io WipperSnapper");

    bootFile.print("Firmware Version: ");
    bootFile.println(WS_VERSION);

    bootFile.print("Board ID: ");
    bootFile.println(BOARD_ID);

#if defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT) ||          \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) || \
    defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)
    bootFile.print("AirLift FW Revision: ");
    bootFile.println(WS._airlift_version);
#endif

    sprintf(sMAC, "%02X:%02X:%02X:%02X:%02X:%02X", WS._macAddr[0],
            WS._macAddr[1], WS._macAddr[2], WS._macAddr[3], WS._macAddr[4],
            WS._macAddr[5]);
    bootFile.print("MAC Address: ");
    bootFile.println(sMAC);

#if PRINT_DEPENDENCIES
    bootFile.println("Build dependencies:");
    bootFile.println(project_dependencies);
#endif

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
void Wippersnapper_FS::createSecretsFile() {
  // Open file for writing
  File32 secretsFile = wipperFatFs.open("/secrets.json", FILE_WRITE);
  secretsFile.truncate(0);
  // Create a default secretsConfig structure
  secretsConfig secretsConfig;
  strcpy(secretsConfig.aio_user, "YOUR_IO_USERNAME_HERE");
  strcpy(secretsConfig.aio_key, "YOUR_IO_KEY_HERE");
  strcpy(secretsConfig.network.ssid, "YOUR_WIFI_SSID_HERE");
  strcpy(secretsConfig.network.pass, "YOUR_WIFI_PASS_HERE");
  secretsConfig.status_pixel_brightness = STATUS_PIXEL_BRIGHTNESS_DEFAULT;

  // Serialize the struct to a JSON document
  JsonDocument doc;
  doc.set(secretsConfig);
  serializeJsonPretty(doc, secretsFile);
  secretsFile.flush();
  secretsFile.close();

  writeToBootOut("ERROR: Please edit the secrets.json file. Then, reset your board.\n");
  // Re-attach the USB device for file access
  delay(500);
  initUSBMSC();
  fsHalt("ERROR: Please edit the secrets.json file. Then, reset your board.");
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::parseSecrets() {
  // Attempt to open the secrets.json file for reading
  File32 secretsFile = wipperFatFs.open("/secrets.json");
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
    convertFromJson(doc["network_type_wifi"], WS._config.network);

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
        convertFromJson(altnetworks[i], WS._multiNetworks[i]);
        WS_DEBUG_PRINT("Added SSID: ");
        WS_DEBUG_PRINTLN(WS._multiNetworks[i].ssid);
        WS_DEBUG_PRINT("PASS: ");
        WS_DEBUG_PRINTLN(WS._multiNetworks[i].pass);
      }
      WS._isWiFiMulti = true;
    } else {
      fsHalt("ERROR: Unrecognised value type for "
             "network_type_wifi.alternative_networks in secrets.json!");
    }
  } else {
    fsHalt("ERROR: Could not find network_type_wifi in secrets.json!");
  }

  // Extract a config struct from the JSON document
  WS._config = doc.as<secretsConfig>();

  // Validate the config struct is not filled with default values
  if (strcmp(WS._config.aio_user, "YOUR_IO_USERNAME_HERE") == 0 ||
      strcmp(WS._config.aio_key, "YOUR_IO_KEY_HERE") == 0) {
    writeToBootOut(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!\n");

    fsHalt(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!");
  }

  if (strcmp(WS._config.network.ssid, "YOUR_WIFI_SSID_HERE") == 0 ||
      strcmp(WS._config.network.pass, "YOUR_WIFI_PASS_HERE") == 0) {
    writeToBootOut("ERROR: Invalid network credentials in secrets.json! TO "
                   "FIX: Please change network_ssid and network_password to "
                   "match your Adafruit IO credentials!\n");

    fsHalt("ERROR: Invalid network credentials in secrets.json! TO FIX: Please "
           "change network_ssid and network_password to match your Adafruit IO "
           "credentials!");
  }

  // specify type of value for json key, by using the |operator to include
  // a typed default value equivalent of with .as<float> w/ default value
  // https://arduinojson.org/v7/api/jsonvariant/or/
  WS._config.status_pixel_brightness =
      doc["status_pixel_brightness"] | (float)STATUS_PIXEL_BRIGHTNESS_DEFAULT;

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
void Wippersnapper_FS::writeToBootOut(PGM_P str) {
  // Append error output to FS
  File32 bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.print(str);
    bootFile.flush();
    bootFile.close();
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to open wipper_boot_out.txt for logging!");
    // feels like we should check why, if good use-case ok, otherwise fsHalt
    // as indicates fs corruption or disc access issue (maybe latter is okay)
  }
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
    @param    msg
                Error message to print to serial console.
*/
/**************************************************************************/
void Wippersnapper_FS::fsHalt(String msg) {
  statusLEDSolid(WS_LED_STATUS_FS_WRITE);
  while (1) {
    WS_DEBUG_PRINTLN("Fatal Error: Halted execution!");
    WS_DEBUG_PRINTLN(msg.c_str());
    delay(1000);
    yield();
  }
}

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
int32_t qspi_msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
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
int32_t qspi_msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

/***************************************************************************/
/*!
    @brief  Callback invoked when WRITE10 command is completed (status
            received and accepted by host). Used to flush any pending cache.
*/
/***************************************************************************/
void qspi_msc_flush_cb(void) {
  // sync w/flash
  flash.syncBlocks();
  // clear file system's cache to force refresh
  wipperFatFs.cacheClear();
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
  return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, /* Physical drive nmuber to identify the drive */
                   const BYTE *buff, /* Data to be written */
                   DWORD sector,     /* Start sector in LBA */
                   UINT count        /* Number of sectors to write */
) {
  (void)pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, /* Physical drive nmuber (0..) */
                   BYTE cmd,  /* Control code */
                   void *buff /* Buffer to send/receive control data */
) {
  (void)pdrv;

  switch (cmd) {
  case CTRL_SYNC:
    flash.syncBlocks();
    return RES_OK;

  case GET_SECTOR_COUNT:
    *((DWORD *)buff) = flash.size() / 512;
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