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
#include "drivers/dispDrv7Seg.h"
#include "drivers/dispDrvBase.h"
#include "drivers/dispDrvBaseI2c.h"
#include "drivers/dispDrvCharLcd.h"
#include "drivers/dispDrvQuadAlphaNum.h"
#include "drivers/dispDrvSh1107.h"
#include "drivers/dispDrvSsd1306.h"
#include "drivers/dispDrvSt7789.h"
#include "wippersnapper.h"
#ifdef ARDUINO_ADAFRUIT_QUALIA_S3_RGB666
#include "drivers/dispDrvRgb666.h"
#endif
#include "drivers/dispDrvThinkInkGrayscale4Eaamfgn.h"
#include "drivers/dispDrvThinkInkGrayscale4MFGN.h"
#include "drivers/dispDrvThinkInkGrayscale4T5.h"
#include "drivers/dispDrvThinkInkMonoAAAMFGN.h"
#include "drivers/dispDrvThinkInkMonoBAAMFGN.h"
#include "drivers/dispDrvThinkInkMonoM06.h"

/*!
    @brief  Interface for interacting with display hardware (TFT, eInk,
            OLED, etc.) in the V2 API.
*/
class DisplayHardware {
public:
  DisplayHardware();
  ~DisplayHardware();
  bool begin(ws_display_Add *addMsg, const char *name);
  bool write(ws_display_Write *msg);
  const char *getName();
  void showSplash();
  void drawStatusBar(const char *io_username);
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_connected);

private:
  char _name[64];
  ws_display_DisplayClass _class =
      ws_display_DisplayClass_DISPLAY_CLASS_UNSPECIFIED;
  dispDrvBase *_drvDisp = nullptr;

  bool beginSpiTft(ws_display_Add *msg);
  bool beginSpiEpd(ws_display_Add *msg);
  bool beginTtlRgb666(ws_display_Add *msg);
  bool beginI2cDisplay(ws_display_Add *msg);
  static int16_t parsePin(const char *pinStr);
  static uint8_t EpdBitBangReadRegister(uint8_t cmd,
                                        ws_display_EpdSpiConfig *config);

  // EPD auto-detection helpers
  bool detect_ssd1680(ws_display_EpdSpiConfig *config);
  bool detect_ssd1683(ws_display_EpdSpiConfig *config);
  bool detect_uc8151d(ws_display_EpdSpiConfig *config);
  bool detect_uc8179(ws_display_EpdSpiConfig *config);
  bool detect_uc8253(ws_display_EpdSpiConfig *config);
};
#endif // WS_DISPLAY_HARDWARE_H
