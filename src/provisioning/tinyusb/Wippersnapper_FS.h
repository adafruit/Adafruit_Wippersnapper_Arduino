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
#ifndef WIPPERSNAPPER_FS_V2_H
#define WIPPERSNAPPER_FS_V2_H

#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include "SdFat_Adafruit_Fork.h"
// using f_mkfs() for formatting
#include "fatfs/ff.h" // NOTE: This should be #included before fatfs/diskio.h!!!
#include "fatfs/diskio.h"

#include "wippersnapper.h"

// forward decl.
class wippersnapper;
struct displayConfig;

// global TinyUSB callbacks
int32_t qspi_msc_write_cb_v2(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
int32_t qspi_msc_read_cb_v2(uint32_t lba, void *buffer, uint32_t bufsize);
void qspi_msc_flush_cb_v2(void);

/*!
    @brief  Class that handles Wippersnapper's optional filesystem commands
                and storage.
*/
class Wippersnapper_FS {
public:
  Wippersnapper_FS();
  ~Wippersnapper_FS();

  void initUSBMSC();

  void GetSDCSPin();

  bool writeFSContents();
  void fsHalt(String msg);
  void fsHalt(String msg, ws_led_status_t ledStatusColor);
  void eraseCPFS();

  bool createBootFile();
  void writeToBootOut(PGM_P str);
  void eraseBootFile();

  // Secrets.json API
  void createSecretsFile();
  bool getSecretsFile();
  void parseSecrets();
#ifdef ARDUINO_FUNHOUSE_ESP32S2
  void parseDisplayConfig(displayConfig &displayFile);
  void createDisplayConfig();
#endif
private:
  bool _is_secrets_file_empty = false;
};
extern wippersnapper Ws;
#endif // Wippersnapper_FS_V2_V2_H