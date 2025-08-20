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

/*!
    @brief  Abstract base class for display drivers.
            This class provides a common interface for all display drivers,
            allowing them to be used interchangeably.
*/
class dispDrvBase {
public:
  /*!
      @brief  Constructor for the base display driver for E-Ink displays.
      @param  dc
              Data/Command pin for the display.
      @param  rst
              Reset pin for the display.
      @param  cs
              Chip Select pin for the display.
      @param  sram_cs
              Optional SRAM Chip Select pin for E-Ink displays that support it.
      @param  busy
              Optional Busy pin for the display.
  */
  dispDrvBase(int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs = -1,
              int16_t busy = -1)
      : _pin_dc(dc), _pin_rst(rst), _pin_cs(cs), _pin_sram_cs(sram_cs),
        _pin_busy(busy) {}

  /*!
      @brief  Destructor for the base display driver.
              This destructor is virtual to allow derived classes to clean up
              resources properly.
  */
  virtual ~dispDrvBase() {}

  /*!
      @brief  Attempts to initialize a ThinkInk EPD driver.
      @param  mode
              The ThinkInk mode to use for the display.
      @param  reset
              Whether to reset the display before initialization.
      @return True if the display was initialized successfully, false otherwise.
  */
  virtual bool begin(thinkinkmode_t mode, bool reset = true);

  /*!
      @brief  Writes a message to the display.
      @param  message
              The message to write to the display.
      @note   MUST be implemented by derived classes.
  */
  virtual void writeMessage(const char *message) = 0;

protected:
  int16_t _pin_dc;      ///< Data/Command pin
  int16_t _pin_rst;     ///< Reset pin
  int16_t _pin_cs;      ///< Chip Select pin
  int16_t _pin_busy;    ///< Optional Busy pin
  int16_t _pin_sram_cs; ///< Optional EPD SRAM chip select pin
  uint8_t _text_sz = 1; ///< Text size for displaying a message
  int16_t _height;      ///< Height of the display
  int16_t _width;       ///< Width of the display
};

#endif // WS_DISP_DRV_BASE_H