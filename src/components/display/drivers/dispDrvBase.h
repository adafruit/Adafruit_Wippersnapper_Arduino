/*!
 * @file src/components/display/drivers/dispDrvBase.h
 *
 * Abstract base class for display drivers.
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
#ifndef WS_DISP_DRV_BASE_H
#define WS_DISP_DRV_BASE_H

#include "Adafruit_ThinkInk.h"
#include "Wippersnapper.h"

class dispDrvBase {
public:
  dispDrvBase(int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs = -1,
              int16_t busy = -1)
      : _pin_dc(dc), _pin_rst(rst), _pin_cs(cs), _pin_sram_cs(sram_cs),
        _pin_busy(busy) {
    // Constructor implementation (if we need one)
  }

  virtual ~dispDrvBase() {
    // Destructor implementation (if we need one)
  };

  // Virtual function to be implemented by derived classes
  virtual bool begin(thinkinkmode_t mode, bool reset = true);

protected:
  int16_t _pin_dc;
  int16_t _pin_rst;
  int16_t _pin_cs;
  int16_t _pin_busy;
  int16_t _pin_sram_cs; // for optional EPD SRAM chip select
};

#endif // WS_DISP_DRV_BASE_H