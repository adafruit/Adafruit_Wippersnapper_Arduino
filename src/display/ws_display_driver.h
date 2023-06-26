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

typedef struct {
  char driver[10]; ///< Display driver type
  int width;       ///< Display width
  int height;      ///< Display height
  int rotation;    ///< Display rotation
  bool isSPI;      ///< Is the display SPI?
  bool isI2C;      ///< Is the display I2C?
  uint8_t pinCS;   ///< Display CS pin
  uint8_t pinDC;   ///< Display DC pin
  uint8_t pinMOSI; ///< Display MOSI pin
  uint8_t pinSCK;  ///< Display SCK pin
  uint8_t pinRST;  ///< Display RST pin
} displayConfig;

LV_FONT_DECLARE(errorTriangle); ///< Error triangle symbol/font

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
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DISPLAY_H