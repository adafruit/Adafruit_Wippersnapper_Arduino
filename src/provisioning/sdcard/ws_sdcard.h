/*!
 * @file ws_sdcard.h
 *
 * Interface for Wippersnapper's SD card filesystem.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SDCARD_H
#define WS_SDCARD_H

#include "SdFat.h"
#include "Wippersnapper_V2.h"
#define SD_FAT_TYPE 3

// forward decl.
class Wippersnapper_V2;

/***************************************************************************/
/*!
    @brief  Class that handles Wippersnapper's optional filesystem commands
                and storage.
*/
/***************************************************************************/
class ws_sdcard {
public:
  ws_sdcard();
  ~ws_sdcard();
  bool IsSDCardInserted();
  bool parseConfigFile();
  bool waitForSerialConfig();
  bool validateJson(const char *input);

private:
  bool _is_sd_card_inserted;  ///< True if an SD card is inserted, False
                              ///< otherwise.
  SdFat _sd;                  ///< SD object from Adafruit SDFat library
  String _serialInput;        ///< Serial input buffer
  const char *json_test_data; ///< Json test data
  bool _use_test_data; ///< True if sample data is being used to test, instead
                       ///< of serial input, False otherwise.
};
extern Wippersnapper_V2 WsV2;
#endif // WS_SDCARD_H