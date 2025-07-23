/*!
 * @file ws_display_driver.h
 *
 * Wippersnapper display driver
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_DISPLAY_H
#define WIPPERSNAPPER_DISPLAY_H

#include "Wippersnapper_V2.h"
#include "provisioning/Config.h"
#include <Adafruit_LvGL_Glue.h> // Always include this BEFORE lvgl.h!
#include <Adafruit_ST7789.h>
#include <lvgl.h>

LV_FONT_DECLARE(errorTriangle); ///< Error triangle symbol/font

class Wippersnapper_V2; // fwd decl

/*!
    @brief  Display driver for LVGL and LVGL_Glue in WipperSnapper.
*/
class ws_display_driver {
public:
  ws_display_driver(){};
  ws_display_driver(displayConfig config);
  ~ws_display_driver();

  bool begin();
  void setResolution(uint16_t displayWidth, uint16_t displayHeight);
  void setRotation(uint8_t rotationMode);
  void enableLogging();
  Adafruit_LvGL_Glue *_glue; ///< LVGL glue object
  void esp32_lvgl_acquire();
  void esp32_lvgl_release();

private:
  Adafruit_ST7789 *_tft_st7789 = nullptr; ///< Adafruit ST7789 display driver
  uint16_t _displayWidth;                 ///< Display width
  uint16_t _displayHeight;                ///< Display height
  uint8_t
      _displayRotationMode; ///< Display rotation (mode, not number in degrees)
};
extern Wippersnapper_V2 WsV2; ///< Global Wippersnapper instance

#endif // WIPPERSNAPPER_DISPLAY_H