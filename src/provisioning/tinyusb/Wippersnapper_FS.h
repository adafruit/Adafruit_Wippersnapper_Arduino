/*!
 * @file Wippersnapper_FS.h
 *
 * Wippersnapper filesystem
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
*/
#ifndef WIPPERSNAPPER_FS_H
#define WIPPERSNAPPER_FS_H

#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
// #include "SdFat.h"
#include "SdFat_Adafruit_Fork.h"
// using f_mkfs() for formatting
#include "fatfs/ff.h" // NOTE: This should be #included before fatfs/diskio.h!!!
#include "fatfs/diskio.h"

#include "Wippersnapper.h"

// forward decl.
class Wippersnapper;
struct displayConfig;

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

  bool initFilesystem(bool force_format = false);
  void initUSBMSC();

  void eraseCPFS();
  void eraseBootFile();

  bool configFileExists();
  void createSecretsFile();
  bool createBootFile();
  void writeToBootOut(PGM_P str);
  void fsHalt(String msg);

  void parseSecrets();

#ifdef ARDUINO_FUNHOUSE_ESP32S2
  void parseDisplayConfig(displayConfig &displayFile);
  void createDisplayConfig();
#endif
private:
  bool _freshFS = false; /*!< True if filesystem was initialized by
                            WipperSnapper, False otherwise. */
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_FS_H