/*!
 * @file Wippersnapper_FS.cpp
 *
 * Wippersnapper Filesystem
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
#if defined(USE_TINYUSB)
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
#else
#error No QSPI/SPI flash are defined on your board variant.h!
#endif

Adafruit_SPIFlash flash(&flashTransport); ///< QSPI flash object
FatFileSystem wipperQSPIFS;               ///< QSPI FatFS object

/**************************************************************************/
/*!
    @brief    Initializes USB-MSC and the QSPI flash filesystem.
*/
/**************************************************************************/
Wippersnapper_FS::Wippersnapper_FS() {
  // detach the USB during initialization
  USBDevice.detach();
  // wait for detach
  delay(50);

  flash.begin();

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

  usb_msc.begin();

  // re-attach the usb device
  USBDevice.attach();
  // wait for enumeration
  delay(500);

  // Init file system on the flash
  wipperQSPIFS.begin(&flash);
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS::~Wippersnapper_FS() {
  io_username = NULL;
  io_key = NULL;
}

/**************************************************************************/
/*!
    @brief    Checks if secrets.json file exists on the flash filesystem.
    @returns  True if secrets.json file exists, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::configFileExists() {
  File secretsFile = wipperQSPIFS.open("/secrets.json");
  if (!secretsFile) {
    WS_DEBUG_PRINTLN("ERROR: Secrets file does not exist on flash.");
    secretsFile.close();
    return false;
  }
  WS_DEBUG_PRINTLN("Found secrets file!");
  secretsFile.close();
  return true;
}

/**************************************************************************/
/*!
    @brief    Creates a skeleton secret.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::createConfigFileSkel() {
  // validate if configuration json file exists on FS
  WS_DEBUG_PRINTLN("Attempting to create secrets file...");
  // open for writing, should create a new file if one doesnt exist
  File secretsFile = wipperQSPIFS.open("/secrets.json", FILE_WRITE);
  if (secretsFile) {
    // Detect which configuration file template via board type //
    // Hardware with built-in AirLift
    if (USB_VID == 0x239A && (USB_PID == 0x8036 || USB_PID == 0x8038)) {
      // Write airlift's secrets.json template to file.
      secretsFile.println(FILE_TEMPLATE_AIRLIFT);
      // done writing, close it
      secretsFile.close();
    } else {
      WS_DEBUG_PRINTLN(
          "ERROR: Your hardware does not support native USB provisioning");
      secretsFile.close();
      while (1)
        yield();
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Could not create secrets.json on QSPI flash...");
    secretsFile.close();
    while (1)
      yield();
  }
  WS_DEBUG_PRINTLN("Successfully added secrets.json to WIPPER volume!");
  WS_DEBUG_PRINTLN("Please edit the secrets.json and reboot your device for "
                   "changes to take effect.");
  while (1)
    yield();
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
    @return   True if parsed successfully, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::parseSecrets() {
  setNetwork = false;
  // open file for parsing
  File secretsFile = wipperQSPIFS.open("/secrets.json");
  if (!secretsFile) {
    WS_DEBUG_PRINTLN("ERROR: Could not open secrets.json file for reading!");
    return false;
  }

  // check if we can deserialize the secrets.json file
  DeserializationError err = deserializeJson(doc, secretsFile);
  if (err) {
    WS_DEBUG_PRINT("ERROR: deserializeJson() failed with code ");
    WS_DEBUG_PRINTLN(err.c_str());
    return false;
  }

  // Get io username
  io_username = doc["io_username"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_username == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_username value in secrets.json!");
    return false;
  }

  // Get io key
  io_key = doc["io_key"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_key == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_key value in secrets.json!");
    return false;
  }

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
      return false;
    }
    // Set WiFi configuration with parsed values
    WS._network_ssid = network_type_wifi_airlift_network_ssid;
    WS._network_pass = network_type_wifi_airlift_network_password;
    setNetwork = true;
  }

  // TODO: We may want to implement network_type_wifi_native (ESP32-S2) here
  // TODO: We may want to implement network_type_ethernet (Ethernet FeatherWing)
  // here

  // Was a network_type detected in the configuration file?
  if (!setNetwork) {
    WS_DEBUG_PRINTLN(
        "ERROR: Network interface not detected in secrets.json file.");
    while (1)
      yield();
  }

  // clear the document and release all memory from the memory pool
  doc.clear();

  // close the tempFile
  secretsFile.close();

  return true;
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
  wipperQSPIFS.cacheClear();
}

#endif // USE_TINYUSB