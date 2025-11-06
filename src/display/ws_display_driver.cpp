/*!
 * @file ws_display_driver.cpp
 *
 * Wippersnapper LVGL Display Driver
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
#ifdef ARDUINO_FUNHOUSE_ESP32S2
#include "ws_display_driver.h"

/*!
    @brief    Creates a new WipperSnapper display driver object from a
              configuration struct.
    @param    config
              Configuration struct., from FS.parseDisplayConfig();
*/
ws_display_driver::ws_display_driver(displayConfig config) {
  // dynamically create the display driver from the configuration file
  if (strcmp(config.driver, "ST7789") == 0) {
    WS_DEBUG_PRINTLN("Creating ST7789 driver");
    // create a new ST7789 driver
    _tft_st7789 = new Adafruit_ST7789((uint8_t)config.spiConfig.pinCs,
                                      (uint8_t)config.spiConfig.pinDc,
                                      (uint8_t)config.spiConfig.pinRst);
  } else {
    Serial.println("ERROR: Display driver type not implemented!");
  }

  // set display resolution and rotation
  setResolution(config.width, config.height);
  setRotation(config.rotation);
}

/*!
    @brief    Deletes a new WipperSnapper display driver object.
*/
ws_display_driver::~ws_display_driver() {
  if (_tft_st7789 != nullptr) {
    delete _tft_st7789;
  }
}

/*!
    @brief    Enables LVGL logging using the usb serial. Must be called
              AFTER calling Serial.begin().
*/
void ws_display_driver::enableLogging() {}

/*!
    @brief    Sets the display's rotation mode.
    @param    rotationMode
              The index for rotation (0-3 inclusive).
*/
void ws_display_driver::setRotation(uint8_t rotationMode) {
  _displayRotationMode = rotationMode;
}

/*!
    @brief    Sets the display resolution, must be called BEFORE begin()!
    @param    displayWidth
              The width of the display, in pixels.
    @param    displayHeight
              The height of the display, in pixels.
*/
void ws_display_driver::setResolution(uint16_t displayWidth,
                                      uint16_t displayHeight) {
  _displayWidth = displayWidth;
  _displayHeight = displayHeight;
}

/*!
    @brief    Initializes the display and the lvgl_glue driver.
    @returns  True if LVGL_Glue began successfully, False otherwise.
*/
bool ws_display_driver::begin() {
  // initialize display driver
  if (_tft_st7789 != nullptr) {
    WS_DEBUG_PRINTLN("Initialize ST7789 driver");
    _tft_st7789->init(_displayWidth, _displayHeight);
  } else {
    Serial.println("ERROR: Unable to initialize the display driver!");
    return false;
  }

// Hardware-specific display commands
#ifdef ARDUINO_FUNHOUSE_ESP32S2
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
#endif // ARDUINO_FUNHOUSE_ESP32S2

  // initialize lvgl_glue
  WS_DEBUG_PRINTLN("Initialize LVGL");
  _glue = new Adafruit_LvGL_Glue();
  LvGLStatus status = _glue->begin(_tft_st7789);
  WS_DEBUG_PRINT("LVGL RC: ");
  WS_DEBUG_PRINTLN((int)status);

  // check if lvgl initialized correctly
  if (status != LVGL_OK) {
    Serial.printf("LVGL_Glue error: %d\r\n", (int)status);
    return false;
  }

  esp32_lvgl_acquire();
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), LV_STATE_DEFAULT);
  esp32_lvgl_release();
  return true;
}

/*!
    @brief    Acquires the LVGL_Glue lock.
*/
void ws_display_driver::esp32_lvgl_acquire() { _glue->lvgl_acquire(); }

/*!
    @brief    Releases the LVGL_Glue lock.
*/
void ws_display_driver::esp32_lvgl_release() { _glue->lvgl_release(); }

#endif