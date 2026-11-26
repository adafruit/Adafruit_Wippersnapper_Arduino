/*!
 * @file src/components/display/drivers/dispDrvThinkInkTricolorZ94.h
 *
 * Driver for ThinkInk 2.9" Tri-Color Z94 display (SSD1680, 296x128).
 * Used by the SSD1680 2.9" tri-color eInk FeatherWing / breakout.
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
#ifndef WS_DRV_THINKINK_TRICOLORZ94_H
#define WS_DRV_THINKINK_TRICOLORZ94_H

#include "dispDrvThinkInkTricolorBase.h"

/*!
    @brief  Driver for a ThinkInk 2.9" Tri-Color Z94 display (SSD1680, 296x128).
*/
class dispDrvThinkInkTricolorZ94 : public dispDrvThinkInkTricolorBase {
public:
  /*!
      @brief  Constructor for the ThinkInk 2.9" Tri-Color Z94 display (SSD1680,
     296x128) driver.
      @param  dc       Data/Command pin.
      @param  rst      Reset pin.
      @param  cs       Chip Select pin.
      @param  sram_cs  Optional SRAM Chip Select pin (-1 for none).
      @param  busy     Optional Busy pin (-1 for none).
  */
  dispDrvThinkInkTricolorZ94(int16_t dc, int16_t rst, int16_t cs,
                             int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvThinkInkTricolorBase(dc, rst, cs, sram_cs, busy) {}

protected:
  /*!
      @brief  Creates and begins the ThinkInk_290_Tricolor_Z94 panel.
      @param  mode  The ThinkInk mode to begin the panel with.
      @return The new panel, or nullptr on failure.
  */
  Adafruit_EPD *createPanel(thinkinkmode_t mode) override {
    ThinkInk_290_Tricolor_Z94 *panel = new ThinkInk_290_Tricolor_Z94(
        _pin_dc, _pin_rst, _pin_cs, _pin_sram_cs, _pin_busy);
    if (panel)
      panel->begin(mode);
    return panel;
  }
};

#endif // WS_DRV_THINKINK_TRICOLORZ94_H
