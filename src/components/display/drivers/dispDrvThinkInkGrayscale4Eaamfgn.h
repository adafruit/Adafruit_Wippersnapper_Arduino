/*!
 * @file src/components/display/drivers/drvDispThinkInkGrayscale4Eaamfgn.h
 *
 * Driver for ThinkInk 2.9" Grayscale 4-level EAAMFGN display (2025 MagTag).
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
#ifndef WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H
#define WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H

#include "dispDrvBase.h"

class drvDispThinkInkGrayscale4Eaamfgn : public dispDrvBase {
public:
  drvDispThinkInkGrayscale4Eaamfgn(int16_t dc, int16_t rst, int16_t cs,
                                   int16_t sram_cs = -1, int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {}

  ~drvDispThinkInkGrayscale4Eaamfgn() {
    if (_display) {
      delete _display;
      _display = nullptr;
    }
  }

  bool begin(thinkinkmode_t mode, bool reset = true) override {
    _display = new ThinkInk_290_Grayscale4_EAAMFGN(_pin_dc, _pin_rst, _pin_cs,
                                                   _pin_sram_cs, _pin_busy);
    if (!_display)
      return false; // Allocation failed

    // Initialize the display
    _display->begin(mode);
    // Clear the display buffer
    _display->clearBuffer();
    _display->display();
    return true;
  }

private:
  ThinkInk_290_Grayscale4_EAAMFGN *_display;
};

#endif // WS_DRV_THINKINK_GRAYSCALE4_EAAMFGN_H