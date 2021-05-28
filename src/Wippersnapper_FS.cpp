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
  WS_DEBUG_PRINTLN("Initializing MSC...")
  _beginMSC();
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS::~Wippersnapper_FS() {}

bool Wippersnapper_FS::parseConfig() { return true; }

bool Wippersnapper_FS::_mountFlashFS() {
  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    WS_DEBUG_PRINTLN("Error, failed to initialize flash chip!");
    return false;
  }
  WS_DEBUG_PRINT("Flash chip JEDEC ID: 0x");
  WS_DEBUG_PRINTLN(flash.getJEDECID(), HEX);

  // Call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!wipperQSPIFS.begin(&flash)) {
    WS_DEBUG_PRINTLN("Error, failed to mount newly formatted filesystem!");
    WS_DEBUG_PRINTLN(
        "Was the flash chip formatted with the SdFat_format example?");
    return false;
  }
  WS_DEBUG_PRINTLN("Mounted filesystem!");
  return true;
}

void Wippersnapper_FS::_beginMSC() {
  // Set disk vendor id, product id and revision with string up to 8, 16, 4
  // characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");

  // Set callbacks
  usb_msc.setReadWriteCallback(qspi_msc_read_cb, qspi_msc_write_cb,
                               qspi_msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.pageSize() * flash.numPages() / 512, 512);

  // Set Lun ready (RAM disk is always ready)
  usb_msc.setUnitReady(true);

  usb_msc.begin();
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t qspi_msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  Serial.printf("QSPI Write block %08x\n", lba);
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t qspi_msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  Serial.printf("QSPI Read block %08x\n", lba);
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). used to flush any pending cache.
void qspi_msc_flush_cb(void) {
  Serial.println("QSPI Flush block");
  // sync w/flash
  flash.syncBlocks();
  // clear file system's cache to force refresh
  wipperQSPIFS.cacheClear();
}