/*!
 * @file Wippersnapper_FS.h
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
#ifndef WIPPERSNAPPER_FS_H
#define WIPPERSNAPPER_FS_H

#include "Wippersnapper.h"

// forward decl.
class Wippersnapper;

int32_t qspi_msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
int32_t qspi_msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);
void qspi_msc_flush_cb(void);

/***************************************************************************/
/*!
    @brief  Class that handles Wippersnapper's optional filesystem commands
                and storage.
*/
/***************************************************************************/
class Wippersnapper_FS {
public:
  Wippersnapper_FS();
  ~Wippersnapper_FS();

  bool _mountFlashFS(); // mounts SPIFlash FS
  bool parseConfig();
  uint32_t getFlashID();

  const char *configNetworkSSID;
  const char *configNetworkPassword;
  const char *configIOUsername;
  const char *configIOKey;

  // File containing the wippersnapper configuration
  FatFile configFile;

  // USB Mass Storage object
  Adafruit_USBD_MSC usb_msc;

  // Holds JSON configuration file
  // NOTE: min. capacity is 192bytes, bumped to 256bytes for longer SSIDs
  // or IO keys
  StaticJsonDocument<256> doc;

private:
};

extern Wippersnapper WS;

#endif // WIPPERSNAPPER_FS_H