/*!
 * @file Wippersnapper_FS.cpp
 *
 * Wippersnapper TinyUSB Filesystem
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#if defined(ARDUINO_MAGTAG29_ESP32S2) || defined(ARDUINO_METRO_ESP32S2) ||     \
    defined(ARDUINO_FUNHOUSE_ESP32S2) ||                                       \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) ||                                  \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
#include "Wippersnapper_FS.h"
// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                           EXTERNAL_FLASH_USE_SPI);
#elif CONFIG_IDF_TARGET_ESP32S2
// ESP32-S2 uses same flash device that stores code.
// Therefore there is no need to specify the SPI and SS
Adafruit_FlashTransport_ESP32 flashTransport;
#else
#error No QSPI/SPI flash are defined on your board variant.h!
#endif

Adafruit_SPIFlash flash(&flashTransport); ///< SPIFlash object
FatFileSystem wipperFatFs;                ///< FatFS object

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
  // Detach USB device during init.
  TinyUSBDevice.detach();
  // Wait for detach
  delay(500);

  // If a filesystem does not already exist - attempt to initialize a new
  // filesystem
  if (!initFilesystem()) {
    WS_DEBUG_PRINTLN("ERROR Initializing Filesystem");
    WS.setStatusLEDColor(RED);
    while (1)
      ;
  }

  // Initialize USB-MSD
  initUSBMSC();

  // If we created a new filesystem, halt until user RESETs device.
  if (_freshFS)
    fsHalt();
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS::~Wippersnapper_FS() {
  // io_username = NULL;
  // io_key = NULL;
}

/**************************************************************************/
/*!
    @brief    Initializes the flash filesystem.
    @return   True if filesystem initialized correctly, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::initFilesystem() {
  // Init. flash library
  if (!flash.begin())
    return false;

  // Check if FS exists
  if (!wipperFatFs.begin(&flash)) {
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
  File writeFile = wipperFatFs.open("/.fseventsd/no_log", FILE_WRITE);
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
    // Create new secrets.json file
    createConfigFileSkel();
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

  // re-attach the usb device
  TinyUSBDevice.attach();
  // wait for enumeration
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
    File libDir = wipperFatFs.open("/lib");
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

  File bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.println("Adafruit.io WipperSnapper");

    bootFile.print("Firmware Version: ");
    bootFile.println(WS_VERSION);

    bootFile.print("Board ID: ");
    bootFile.println(BOARD_ID);

    bootFile.print("Firmware Version: ");
    bootFile.println(WS_VERSION);

    sprintf(sMAC, "%02X:%02X:%02X:%02X:%02X:%02X", WS._macAddr[0],
            WS._macAddr[1], WS._macAddr[2], WS._macAddr[3], WS._macAddr[4],
            WS._macAddr[5]);
    bootFile.print("MAC Address: ");
    bootFile.println(sMAC);

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
    @brief    Creates a skeleton secret.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::createConfigFileSkel() {
  // open for writing, create a new file if one doesnt exist
  File secretsFile = wipperFatFs.open("/secrets.json", FILE_WRITE);
  if (!secretsFile) {
    WS.setStatusLEDColor(RED);
    while (1)
      yield();
  }
  // write out secrets file to USB-MSD
  // NOTE: This was chunked into sep. lines, had issue with writing the entire
  // file at once
  secretsFile.print("{\n\t\"io_username\":\"YOUR_IO_USERNAME_HERE\",\n\t\"io_"
                    "key\":\"YOUR_IO_KEY_");
  secretsFile.flush();
// platform-dependent changes
#if defined(ARDUINO_MAGTAG29_ESP32S2) || defined(ARDUINO_METRO_ESP32S2) ||     \
    defined(ARDUINO_FUNHOUSE_ESP32S2) ||                                       \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) ||                                  \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  secretsFile.print("HERE\",\n\t\"network_type_wifi_native\":{\n\t\t\"network_"
                    "ssid\":\"YOUR_WIFI_SSID_");
#elif defined(ADAFRUIT_PYPORTAL) || defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
  secretsFile.print("HERE\",\n\t\"network_type_wifi_airlift\":{\n\t\t\"network_"
                    "ssid\":\"YOUR_WIFI_SSID_");
#else
  secretsFile.println("!!! ERROR: undefined board !!!");
#endif
  secretsFile.flush();
  secretsFile.print(
      "HERE\",\n\t\t\"network_password\":\"YOUR_WIFI_PASS_HERE\"\n\t}\n}");
  secretsFile.flush();
  secretsFile.close();
  writeToBootOut(
      "* Please edit the secrets.json file. Then, reset your board.");
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::parseSecrets() {
  setNetwork = false;
  // open file for parsing
  File secretsFile = wipperFatFs.open("/secrets.json");
  if (!secretsFile) {
    WS_DEBUG_PRINTLN("ERROR: Could not open secrets.json file for reading!");
    fsHalt();
  }

  // check if we can deserialize the secrets.json file
  DeserializationError err = deserializeJson(doc, secretsFile);
  if (err) {
    WS_DEBUG_PRINT("ERROR: deserializeJson() failed with code ");
    WS_DEBUG_PRINTLN(err.c_str());

    writeToBootOut("ERROR: deserializeJson() failed with code");
    writeToBootOut(err.c_str());
    fsHalt();
  }

  // Get io username
  const char *io_username = doc["io_username"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_username == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_username value in secrets.json!");
    writeToBootOut("ERROR: invalid io_username value in secrets.json!");
    while (1) {
      WS.statusLEDBlink(WS_LED_STATUS_FS_WRITE);
      yield();
    }
  }

  // check if username is from templated json
  if (doc["io_username"] == "YOUR_IO_USERNAME_HERE") {
    writeToBootOut(
        "* ERROR: Default username found in secrets.json, please edit "
        "the secrets.json file and reset the board for the changes to take "
        "effect");
    fsHalt();
  }
  WS._username = io_username;

  writeToBootOut("Adafruit.io Username: ");
  writeToBootOut(WS._username);

  // Get io key
  const char *io_key = doc["io_key"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_key == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_key value in secrets.json!");
    writeToBootOut("ERROR: invalid io_key value in secrets.json!");
    fsHalt();
  }
  WS._key = io_key;

  // next, we detect the network interface from the `secrets.json`
  WS_DEBUG_PRINTLN("Attempting to find network interface...");
  // Check if type is WiFi (AirLift)
  const char *network_type_wifi_airlift_network_ssid =
      doc["network_type_wifi_airlift"]["network_ssid"];
  if (network_type_wifi_airlift_network_ssid == nullptr) {
    WS_DEBUG_PRINTLN(
        "Network interface is not airlift type, checking next type...");
  } else {
    WS_DEBUG_PRINTLN("Network Interface Found: AirLift ESP32 Co-Processor");
    WS_DEBUG_PRINTLN("Setting network:");
    // Parse network password
    const char *network_type_wifi_airlift_network_password =
        doc["network_type_wifi_airlift"]["network_password"];
    // validation
    if (network_type_wifi_airlift_network_password == nullptr) {
      WS_DEBUG_PRINTLN(
          "ERROR: invalid network_type_wifi_airlift_network_password value in "
          "secrets.json!");
      writeToBootOut(
          "ERROR: invalid network_type_wifi_airlift_network_password value in "
          "secrets.json!");
      fsHalt();
    }
    // check if SSID is from template (not entered)
    if (doc["network_type_wifi_airlift"]["network_password"] ==
        "YOUR_WIFI_SSID_HERE") {
      writeToBootOut("Default SSID found in secrets.json, please edit "
                     "the secrets.json file and reset the board");
      fsHalt();
    }

    // Set WiFi configuration with parsed values
    WS._network_ssid = network_type_wifi_airlift_network_ssid;
    WS._network_pass = network_type_wifi_airlift_network_password;
    setNetwork = true;
  }

  // Check if type is WiFi (Native - ESP32-S2, ESP32)
  const char *network_type_wifi_native_network_ssid =
      doc["network_type_wifi_native"]["network_ssid"];
  if (network_type_wifi_native_network_ssid == nullptr) {
    WS_DEBUG_PRINTLN(
        "Network interface is not native WiFi, checking next type...");
  } else {
    WS_DEBUG_PRINTLN("Network Interface Found: Native WiFi (ESP32S2, ESP32)");
    WS_DEBUG_PRINTLN("Setting network:");
    // Parse network password
    const char *network_type_wifi_native_network_password =
        doc["network_type_wifi_native"]["network_password"];
    // validation
    if (network_type_wifi_native_network_password == nullptr) {
      WS_DEBUG_PRINTLN(
          "ERROR: invalid network_type_wifi_native_network_password value in "
          "secrets.json!");
      writeToBootOut(
          "ERROR: invalid network_type_wifi_native_network_password value in "
          "secrets.json!");
      fsHalt();
    }
    // check if SSID is from template (not entered)
    if (doc["network_type_wifi_native"]["network_password"] ==
        "YOUR_WIFI_SSID_HERE") {
      writeToBootOut("Default SSID found in secrets.json, please edit "
                     "the secrets.json file and reset the board");
      fsHalt();
    }

    // Set WiFi configuration with parsed values
    WS._network_ssid = network_type_wifi_native_network_ssid;
    WS._network_pass = network_type_wifi_native_network_password;
    setNetwork = true;
  }

  // Was a network_type detected in the configuration file?
  if (!setNetwork) {
    WS_DEBUG_PRINTLN(
        "ERROR: Network interface not detected in secrets.json file.");
    writeToBootOut(
        "ERROR: Network interface not detected in secrets.json file.");
    fsHalt();
  }

  writeToBootOut("WiFi Network: ");
  writeToBootOut(WS._network_ssid);

  // Optional, Set the IO URL
  WS._mqttBrokerURL = doc["io_url"];

  // clear the document and release all memory from the memory pool
  doc.clear();

  // close the tempFile
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
  File bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.println(str);
    bootFile.flush();
    bootFile.close();
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to open wipper_boot_out.txt for logging!");
  }
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
*/
/**************************************************************************/
void Wippersnapper_FS::fsHalt() {
  while (1) {
    WS.statusLEDBlink(WS_LED_STATUS_FS_WRITE);
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