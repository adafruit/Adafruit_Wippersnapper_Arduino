/*!
 * @file src/components/display/hardware.h
 *
 * Hardware interface for display components (V2).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISPLAY_HARDWARE_H
#define WS_DISPLAY_HARDWARE_H
#include "Wippersnapper.h"
#include "drivers/dispDrvBase.h"
#include "drivers/dispDrvSt7789.h"
#include <string.h>

/*!
    @brief  Interface for interacting with display hardware (TFT, eInk,
            OLED, etc.) in the V2 API.
*/
class DisplayHardware {
public:
  DisplayHardware();
  ~DisplayHardware();

  bool begin(ws_display_Add *addMsg);
  bool write(ws_display_Write *msg);
  const char *getName();

  void showSplash();
  void drawStatusBar(const char *io_username);
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_connected);

private:
  char _name[64];
  ws_display_DisplayType _type;
  dispDrvBase *_drvDisp = nullptr;

  bool beginSpiTft(ws_display_Add *msg);
  int16_t parsePin(const char *pinStr);
};
#endif // WS_DISPLAY_HARDWARE_H
