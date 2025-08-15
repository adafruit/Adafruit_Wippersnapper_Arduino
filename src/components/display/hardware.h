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

#include "Adafruit_ThinkInk.h"

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
  DisplayHardware();
  ~DisplayHardware();
  // High-level API functions
  void setType(wippersnapper_display_v1_DisplayType type);
  bool begin(bool reset = true);
  // API functions to abstract Adafruit_GFX
  void setTextSize(uint8_t sz);

  private:
    wippersnapper_display_v1_DisplayType _type; ///< Display type
    // TODO: Make these drivers instead?
    ThinkInk_290_Grayscale4_EAAMFGN *disp_thinkink_grayscale4_eaamfgn = nullptr; //< 2025 MagTag with SSD1680Z chipset
    ThinkInk_290_Grayscale4_T5 *disp_thinkink_grayscale4_t5 = nullptr; ///< Pre-2025 MagTag with IL0373 chipset
};
#endif // WS_DISPLAY_HARDWARE_H