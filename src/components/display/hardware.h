/*!
 * @file src/components/display/hardware.h
 *
 * Hardware interface for display components.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISPLAY_HARDWARE_H
#define WS_DISPLAY_HARDWARE_H
#include "Wippersnapper.h"
#include "drivers/dispDrvBase.h"
#include "drivers/dispDrvThinkInkGrayscale4Eaamfgn.h"

/**************************************************************************/
/*!
    @brief  Interface for interacting with display hardware (TFT, eInk,
            OLED, etc.)
            This class provides methods to initialize, write to, and
            manage the state of display hardware.
*/
/**************************************************************************/
class DisplayHardware {
public:
  DisplayHardware(const char *name);
  ~DisplayHardware();
  // High-level API functions
  const char *getName();
  void setType(wippersnapper_display_v1_DisplayType type);
  wippersnapper_display_v1_DisplayType getType();
  bool beginEPD(wippersnapper_display_v1_EPDConfig *config,
                wippersnapper_display_v1_EpdSpiConfig *spi_config);
  bool begin(bool reset = true);
  // API functions to abstract Adafruit_GFX
  void setTextSize(uint8_t sz);

private:
  const char *_name; ///< Identifies the hardware instance
  wippersnapper_display_v1_DisplayType _type; ///< Display type
  wippersnapper_display_v1_EPDThinkInkPanel
      _thinkink_driver; ///< ThinkInk driver type
  drvDispThinkInkGrayscale4Eaamfgn *_disp_thinkink_grayscale4_eaamfgn = nullptr;
};
#endif // WS_DISPLAY_HARDWARE_H