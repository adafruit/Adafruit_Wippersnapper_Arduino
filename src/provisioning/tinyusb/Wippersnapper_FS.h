/*!
 * @file Wippersnapper_FS.h
 *
 * Wippersnapper filesystem
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2025 for Adafruit Industries.
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

#include "Wippersnapper_V2.h"

// forward decl.
class Wippersnapper_V2;
struct displayConfig;

// global TinyUSB callbacks
int32_t qspi_msc_write_cb_v2(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
int32_t qspi_msc_read_cb_v2(uint32_t lba, void *buffer, uint32_t bufsize);
void qspi_msc_flush_cb_v2(void);
bool msc_ready_callback(void);
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
  void InitUsbMsc();
  // Filesystem
  bool MakeDefaultFilesystem();
  void HaltFilesystem(String msg);
  void HaltFilesystem(String msg, ws_led_status_t ledStatusColor);
  void EraseCircuitPythonFS();
  void USBDetach();
  void USBAttach();
  // boot.txt
  bool CreateFileBoot();
  void WriteFileBoot(PGM_P str);
  void EraseFileBoot();
  // secrets.json
  void CreateFileSecrets();
  bool GetFileSecrets();
  void ParseFileSecrets();
#ifdef ARDUINO_FUNHOUSE_ESP32S2
  void ParseFileDisplayCfg(displayConfig &displayFile);
  void CreateDisplayCfg();
#endif
  // config.json
  void CreateFileConfig();
  bool AddSDCSPinToFileConfig(uint8_t pin);
  void GetPinSDCS();
  bool AddI2CDeviceToConfig(uint32_t address);

private:
  bool _is_secrets_file_empty = false;
};
extern Wippersnapper_V2 WsV2;
#endif // Wippersnapper_FS_H