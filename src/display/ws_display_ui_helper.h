/*!
 * @file ws_display_ui_helper.h
 *
 * LVGL "helper" class for WipperSnapper.
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

#ifndef WS_DISPLAY_UI_HELPER_H
#define WS_DISPLAY_UI_HELPER_H

#include "Wippersnapper.h"
#include <lvgl.h>
#include "ws_display_driver.h"
#include "ws_display_tooltips.h"

// External Fonts
#define SYMBOL_CODE "\xEF\x87\x89"            ///< Symbol code for file icon
#define SYMBOL_WIFI "\xEF\x87\xAB"            ///< Symbol code for WiFi icon
#define SYMBOL_TURTLE30PX "\xEF\x9C\xA6"      ///< Symbol code for turtle icon
#define SYMBOL_CLOUD "\xEF\x83\x82"           ///< Symbol code for cloud icon
#define SYMBOL_ERROR_TRIANGLE "\xEF\x81\xB1"  ///< Symbol code for error triangle icon
LV_FONT_DECLARE(errorTriangle);
LV_FONT_DECLARE(file);
LV_FONT_DECLARE(wifi_30px);
LV_FONT_DECLARE(cloud_30px);
LV_FONT_DECLARE(turtle_30px);
LV_FONT_DECLARE(circle_30px);

// Images
LV_IMG_DECLARE(ws_icon_100px);

/* Screen: Loading */
// Objects
static lv_obj_t *imgWSLogo;
static lv_obj_t *lblIconFile;
static lv_obj_t *lblIconWiFi;
static lv_obj_t *labelTurtleBar;
static lv_obj_t *labelCloudBar;
static lv_obj_t *labelCircleBar;
static lv_obj_t *lblStatusText;
static lv_obj_t *lblTipText;
// Styles
static lv_style_t styleIconFile;
static lv_style_t styleIconWiFi;
static lv_style_t styleIconTurtle30px;
static lv_style_t styleIconCloud;
static lv_style_t styleIconCheckmark;

/* Screen: Error */
// Objects
static lv_obj_t *labelErrorTriangle;
static lv_obj_t *labelErrorHeader;
static lv_obj_t *labelErrorBody;
// Styles
static lv_style_t styleErrorTriangle;
static lv_style_t styleLabelErrorLarge;
static lv_style_t styleLabelErrorSmall;

enum loadBarIcons {
  loadBarIconFile,
  loadBarIconWifi,
  loadBarIconCloud,
  loadBarIconTurtle,
  loadBarIconCheckmark
}; ///< Icon names for use by set_load_bar_icon_complete

// holds all the loading tips
static const char* loading_tips[4] = { WS_LOADING_TIP_1, WS_LOADING_TIP_2, WS_LOADING_TIP_3, WS_LOADING_TIP_4 };

static lv_timer_t * timerLoadTips;

/**************************************************************************/
/*!
    @brief    Helps build and manage the LVGL objects and screens for
                the application code.
*/
/**************************************************************************/
class ws_display_ui_helper {
public:
  ws_display_ui_helper(){};
  ~ws_display_ui_helper(){};

  void set_bg_black();

  void show_scr_load();
  void clear_scr_load();
  void set_load_bar_icon_complete(loadBarIcons iconType);
  void set_label_status(const char *text); // callback ui help?
  void remove_tip_timer();

  void show_scr_error(const char *lblError, const char *lblDesc);
};
#endif // WS_DISPLAY_UI_HELPER_H
