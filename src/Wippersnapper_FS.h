/*!
 * @file Wippersnapper_FS.h
 *
 * Wippersnapper filesystem
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

// global TinyUSB callbacks
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

  bool parseSecrets();
  bool configFileExists();
  void createConfigFileSkel();

  // Adafruit IO Configuration
  const char *io_username = NULL;
  const char *io_key = NULL;

  bool setNetwork;

  File secretsFile;  // File object to hold the contents of secrets.json

  // USB Mass Storage object
  Adafruit_USBD_MSC usb_msc;

  // Holds JSON configuration file
  // NOTE: calculated capacity with
  // maximum length of usernames/passwords/tokens
  // is 382 bytes, rounded to nearest power of 2.
  StaticJsonDocument<512> doc;

private:
};

extern Wippersnapper WS;

#endif // WIPPERSNAPPER_FS_H