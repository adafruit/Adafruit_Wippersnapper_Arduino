/*!
 * @file ws_display_ui_helper.h
 *
 * LVGL UI Helper class for WipperSnapper
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
#include "ws_display_driver.h"
#include "ws_display_tooltips.h"
#include <lvgl.h>

/**********************
 *      MACROS
 **********************/
#define MAX_CONSOLE_TEXT_LEN 430 ///< Maximum text length on the console
/* External fonts and symbols */
#define SYMBOL_CODE "\xEF\x87\x89"       ///< Symbol code for file icon
#define SYMBOL_WIFI "\xEF\x87\xAB"       ///< Symbol code for WiFi icon
#define SYMBOL_TURTLE30PX "\xEF\x9C\xA6" ///< Symbol code for turtle icon
#define SYMBOL_CLOUD "\xEF\x83\x82"      ///< Symbol code for cloud icon
#define SYMBOL_ERROR_TRIANGLE                                                  \
  "\xEF\x81\xB1" ///< Symbol code for error triangle icon

/**********************
 *  STATIC VARIABLES
 **********************/
/* Loading screen */
static lv_obj_t *imgWSLogo;
static lv_obj_t *lblIconFile;
static lv_obj_t *lblIconWiFi;
static lv_obj_t *labelTurtleBar;
static lv_obj_t *labelCloudBar;
static lv_obj_t *lblStatusText;
static lv_obj_t *lblTipText;
static lv_style_t styleIconFile;
static lv_style_t styleIconWiFi;
static lv_style_t styleIconTurtleStatusbar;
static lv_style_t styleIconCloud;
static lv_style_t styleIconCheckmark;

/* Error screen */
static lv_obj_t *labelErrorTriangle;
static lv_obj_t *labelErrorHeader;
static lv_obj_t *labelErrorBody;
static lv_style_t styleErrorTriangle;
static lv_style_t styleLabelErrorLarge;
static lv_style_t styleLabelErrorSmall;

/* Monitor screen */
static lv_obj_t *canvasStatusBar;
static lv_draw_rect_dsc_t *rect_dsc;
static lv_obj_t *statusbar_icon_bat;
static lv_obj_t *statusbar_icon_wifi;
static lv_obj_t *terminalLabel;
static lv_style_t styleTerminalLabel;

/**********************
 *  IMAGE DECLARE
 **********************/
LV_FONT_DECLARE(errorTriangle); ///< Error triangle icon
LV_FONT_DECLARE(file);          ///< File icon
LV_FONT_DECLARE(wifi_30px);     ///< WiFi icon
LV_FONT_DECLARE(cloud_30px);    ///< Cloud icon
LV_FONT_DECLARE(turtle_30px);   ///< Turtle icon
LV_FONT_DECLARE(turtle_16);     ///< Turtle icon

/**********************
 *  Timers
 **********************/
static lv_timer_t *timerLoadTips;

/// Icon names for use with set_load_bar_icon_complete()
enum loadBarIcons {
  loadBarIconFile,     ///< File icon
  loadBarIconWifi,     ///< WiFi icon
  loadBarIconCloud,    ///< Cloud icon
  loadBarIconTurtle,   ///< Turtle icon
  loadBarIconCheckmark ///< Checkmark icon
};

static const char *loading_tips[4] = {
    WS_LOADING_TIP_1, WS_LOADING_TIP_2, WS_LOADING_TIP_3,
    WS_LOADING_TIP_4}; ///< Holds the loading "tips"
static char terminalTextBuffer[MAX_CONSOLE_TEXT_LEN +
                               1]; ///< Contains all text actively displayed on
                                   ///< the terminal

class ws_display_driver;

/**************************************************************************/
/*!
    @brief    Helps build and manage the LVGL objects and screens for
                the application code.
*/
/**************************************************************************/
class ws_display_ui_helper {
public:
  ws_display_ui_helper(ws_display_driver *drv) { _dispDriver = drv; };
  ~ws_display_ui_helper(){};

  void set_bg_black();
  void show_scr_load();
  void clear_scr_load();
  void build_scr_monitor();
  void add_text_to_terminal(const char *text);
  void set_load_bar_icon_complete(loadBarIcons iconType);
  void set_label_status(const char *text); // callback ui help?
  void remove_tip_timer();
  void show_scr_error(const char *lblError, const char *lblDesc);
  bool getLoadingState();

private:
  ws_display_driver *_dispDriver = nullptr;
  void addToTerminal(const char *txt_in);
  bool _loadingState = false;
};
#endif // WS_DISPLAY_UI_HELPER_H