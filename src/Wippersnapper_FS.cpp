/*!
 * @file Wippersnapper_FS.cpp
 *
 * Wippersnapper TinyUSB filesystem
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

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
// Flash object
Adafruit_SPIFlash flash(&flashTransport);
// QSPI FATFS object
FatFileSystem wipperQSPIFS;

/**************************************************************************/
/*!
    @brief    Creates a new instance of a Filesystem object.
    @param    *ws
              Reference to Wippersnapper.
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
Wippersnapper_FS::~Wippersnapper_FS() {}

uint32_t Wippersnapper_FS::getFlashID() { return flash.getJEDECID(); }

bool Wippersnapper_FS::configFileExists() {
  secretsFile = wipperQSPIFS.open("/secrets.json");
  if (!secretsFile) {
    WS_DEBUG_PRINTLN("Secrets file does not exist on flash.");
    secretsFile.close();
    return false;
  }
  WS_DEBUG_PRINTLN("Found secrets file!");
  secretsFile.close();
  return true;
}

void Wippersnapper_FS::createConfigFileSkel() {
  // open for writing, should create a new file if one doesnt exist
  secretsFile = wipperQSPIFS.open("/secrets.json", FILE_WRITE);
  if (!secretsFile) {
    Serial.println("ERROR: Could not create secrets.json on QSPI flash...");
    while (1)
      yield();
  }
  // write json string to file
  secretsFile.print("{\"network_ssid\":\"YOUR_WIFI_SSID_HERE\",\"network_"
                    "password\":\"YOUR_WIFI_PASS_HERE\",\"io_username\":\"YOUR_"
                    "IO_USERNAME_HERE\",\"io_key\":\"YOUR_IO_KEY_HERE\"}");
  // done writing, close file
  secretsFile.close();
  Serial.println("Created secrets.json file on flash");
}

bool Wippersnapper_FS::parseSecrets() {
  // open file for parsing
  secretsFile = wipperQSPIFS.open("/secrets.json");
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
  const char *io_username = doc["io_username"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_username == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_username in JSON document!");
    return false;
  }

  // Get io key
  const char *io_key = doc["io_key"];
  // error check against default values [ArduinoJSON, 3.3.3]
  if (io_key == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: invalid io_key in JSON document!");
    return false;
  }

  WS_DEBUG_PRINT("Found AIO USER: "); WS_DEBUG_PRINTLN(io_username);
  WS_DEBUG_PRINT("Found AIO Key: "); WS_DEBUG_PRINTLN(io_key);

  // Set AIO username
  WS._username = io_username;
  // Set AIO key
  WS._key = io_key;

  // clear the document and release all memory from the memory pool
  doc.clear();

  // close the tempFile
  secretsFile.close();

  return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t qspi_msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t qspi_msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). used to flush any pending cache.
void qspi_msc_flush_cb(void) {
  // sync w/flash
  flash.syncBlocks();
  // clear file system's cache to force refresh
  wipperQSPIFS.cacheClear();
}