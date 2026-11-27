/*!
 * @file src/components/display/drivers/dispDrvThinkInkTricolorZ17.h
 *
 * Driver for ThinkInk 1.54" Tri-Color Z17 display (IL0373, 152x152).
 * Used by the 1.54" 152x152 tri-color eInk breakout (ADA3625).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DRV_THINKINK_TRICOLORZ17_H
#define WS_DRV_THINKINK_TRICOLORZ17_H

#include "dispDrvThinkInkTricolorBase.h"

/*!
    @brief  Driver for a ThinkInk 1.54" Tri-Color Z17 display (IL0373, 152x152).
*/
class dispDrvThinkInkTricolorZ17 : public dispDrvThinkInkTricolorBase {
public:
  /*!
      @brief  Constructor for the ThinkInk 1.54" Tri-Color Z17 display (IL0373,
     152x152) driver.
      @param  dc       Data/Command pin.
      @param  rst      Reset pin.
      @param  cs       Chip Select pin.
      @param  sram_cs  Optional SRAM Chip Select pin (-1 for none).
      @param  busy     Optional Busy pin (-1 for none).
  */
  dispDrvThinkInkTricolorZ17(int16_t dc, int16_t rst, int16_t cs,
                             int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvThinkInkTricolorBase(dc, rst, cs, sram_cs, busy) {}

protected:
  /*!
      @brief  Creates and begins the ThinkInk_154_Tricolor_Z17 panel.
      @param  mode  The ThinkInk mode to begin the panel with.
      @return The new panel, or nullptr on failure.
  */
  Adafruit_EPD *createPanel(thinkinkmode_t mode) override {
    ThinkInk_154_Tricolor_Z17 *panel = new ThinkInk_154_Tricolor_Z17(
        _pin_dc, _pin_rst, _pin_cs, _pin_sram_cs, _pin_busy);
    if (panel)
      panel->begin(mode);
    return panel;
  }
};

#endif // WS_DRV_THINKINK_TRICOLORZ17_H
