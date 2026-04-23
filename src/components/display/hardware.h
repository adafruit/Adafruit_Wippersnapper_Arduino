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
#include "drivers/dispDrvBase.h"
#include "drivers/dispDrvBaseI2c.h"
#include "drivers/dispDrvSt7789.h"
#include "drivers/drvOut7Seg.h"
#include "drivers/drvOutCharLcd.h"
#include "drivers/drvOutQuadAlphaNum.h"
#include "drivers/drvOutSh1107.h"
#include "drivers/drvOutSsd1306.h"
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

  /*!
      @brief  Initializes the underlying display driver from a Display Add
              message.
      @param  addMsg  The Display Add message describing the display.
      @param  name    The unique name for this display instance.
      @return True if initialization succeeded, False otherwise.
  */
  bool begin(ws_display_Add *addMsg, const char *name);
  /*!
      @brief  Writes a Display Write message to the display.
      @param  msg  The Display Write message to render.
      @return True if the write succeeded, False otherwise.
  */
  bool write(ws_display_Write *msg);
  /*!
      @brief  Returns the name of this display instance.
      @return Pointer to the display's name string.
  */
  const char *getName();

  /*! @brief Shows the driver's splash screen, if supported. */
  void showSplash();
  /*!
      @brief  Draws the status bar and the Adafruit IO username.
      @param  io_username  Adafruit IO username to display.
  */
  void drawStatusBar(const char *io_username);
  /*!
      @brief  Updates status bar icons based on current state.
      @param  rssi            Current WiFi RSSI value.
      @param  bat             Current battery level (0-100).
      @param  mqtt_connected  True if MQTT is connected.
  */
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_connected);

private:
  char _name[64];
  ws_display_DisplayClass _type;
  dispDrvBase *_drvDisp = nullptr;

  bool beginSpiTft(ws_display_Add *msg);
  bool beginSpiEpd(ws_display_Add *msg);
  bool beginTtlRgb666(ws_display_Add *msg);
  bool beginI2cDisplay(ws_display_Add *msg);
  int16_t parsePin(const char *pinStr);

  // EPD auto-detection helpers
  bool detect_ssd1680(uint8_t cs, uint8_t dc, uint8_t rst);
  bool detect_ssd1683(uint8_t cs, uint8_t dc, uint8_t rst);
  bool detect_uc8151d(uint8_t cs, uint8_t dc, uint8_t rst);
  bool detect_uc8179(uint8_t cs, uint8_t dc, uint8_t rst);
  bool detect_uc8253(uint8_t cs, uint8_t dc, uint8_t rst);
};
#endif // WS_DISPLAY_HARDWARE_H
