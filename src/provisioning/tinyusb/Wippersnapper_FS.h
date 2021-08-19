/*!
 * @file Wippersnapper_FS.h
 *
 * Wippersnapper filesystem
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
#ifndef WIPPERSNAPPER_FS_H
#define WIPPERSNAPPER_FS_H

#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include "SdFat.h"
// using f_mkfs() for formatting
#include "fatfs/ff.h"
#include "fatfs/diskio.h"

#include "Wippersnapper.h"

#define VOLUME_LABEL "WIPPER" ///< FatFs volume label
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
  bool createBootFile();
  void writeErrorToBootOut(PGM_P str);

  // Adafruit IO Configuration
  const char *io_username =
      NULL;                  /*!< Adafruit IO username, from config json. */
  const char *io_key = NULL; /*!< Adafruit IO password, from config json. */
  bool setNetwork; /*!< True if a network interface type was set up, False
                      otherwise. */

  // NOTE: calculated capacity with maximum
  // length of usernames/passwords/tokens
  // is 382 bytes, rounded to nearest power of 2.
  StaticJsonDocument<512> doc; /*!< Json configuration file */
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_FS_H