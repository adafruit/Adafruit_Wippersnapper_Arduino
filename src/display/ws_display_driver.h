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

#include "Wippersnapper.h"

#include <Adafruit_LvGL_Glue.h> // Always include this BEFORE lvgl.h!
#include <Adafruit_ST7789.h>
#include <lvgl.h>

struct displayConfig {
  char driver[10];
  int width;
  int height;
  int rotation;
  bool isSPI;
  bool isI2C;
  uint8_t pinCS;
  uint8_t pinDC;
  uint8_t pinMOSI;
  uint8_t pinSCK;
  uint8_t pinRST;
};

LV_FONT_DECLARE(errorTriangle);

class Wippersnapper; // fwd decl

/***************************************************************************/
/*!
    @brief  Display driver for LVGL and LVGL_Glue in WipperSnapper.
*/
/***************************************************************************/
class ws_display_driver {
public:
  ws_display_driver(){};
  ws_display_driver(displayConfig config);
  ~ws_display_driver();

  bool begin();
  void setResolution(uint16_t displayWidth, uint16_t displayHeight);
  void setRotation(uint8_t rotationMode);
  void enableLogging();
  Adafruit_LvGL_Glue *_glue;
  void esp32_lvgl_acquire();
  void esp32_lvgl_release();
private:
  
  Adafruit_ST7789 *_tft_st7789 = nullptr;
  uint16_t _displayWidth;
  uint16_t _displayHeight;
  uint8_t _displayRotationMode;
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DISPLAY_H