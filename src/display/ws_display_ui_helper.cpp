/*!
 * @file ws_display_ui_helper.cpp
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

#include "ws_display_ui_helper.h"


/**************************************************************************/
/*!
    @brief    Changes a label every 2 seconds to a new, random, tip.
    @param    timer
              The lv_timer tied to this callback, timerLoadTips.
*/
/**************************************************************************/
void lv_timer_tips_cb(lv_timer_t *timer) {
  Serial.println("Timer tips cb called");
  long tipNum = random(0, sizeof(loading_tips) / sizeof(loading_tips[0]));
  lv_label_set_text(lblTipText, loading_tips[tipNum]);
}



/**************************************************************************/
/*!
    @brief    Callback for updating the status label on the loading screen.
    @param    event
              Callback data.
*/
/**************************************************************************/
static void label_status_cb(lv_event_t *event) {
  Serial.println("eventcb called!");
  const char **charPtr{static_cast<const char **>(lv_event_get_param(event))};
  Serial.print("text: ");
  Serial.println(*charPtr);
  lv_label_set_text(lblStatusText, *charPtr);
}

/**************************************************************************/
/*!
    @brief    Sets the text of the status label on the loading screen.
    @param    text
              Desired text to write to the status label.
*/
/**************************************************************************/
void ws_display_ui_helper::set_label_status(const char *text) {
  Serial.print("set_label_status (text): ");
  Serial.println(text);
  _dispDriver->esp32_lvgl_acquire();
  lv_event_send(lblStatusText, LV_EVENT_REFRESH, &text);
  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Pauses and deletes the loading tip callback timer.
*/
/**************************************************************************/
void ws_display_ui_helper::remove_tip_timer() {
  lv_timer_pause(timerLoadTips);
  lv_timer_del(timerLoadTips);
}

/**************************************************************************/
/*!
    @brief    Sets the screen's background to a black color.
*/
/**************************************************************************/
void ws_display_ui_helper::set_bg_black() {
  _dispDriver->esp32_lvgl_acquire();
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_STATE_DEFAULT);
  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Sets the color of an icon on the loading screen to green.
    @param    iconType
              Desired icon.
*/
/**************************************************************************/
void ws_display_ui_helper::set_load_bar_icon_complete(loadBarIcons iconType) {
  static lv_style_t *styleIcon;
  static lv_obj_t *objIcon;

  switch (iconType) {
  case loadBarIconFile:
    styleIcon = &styleIconFile;
    objIcon = lblIconFile;
    break;
  case loadBarIconWifi:
    styleIcon = &styleIconWiFi;
    objIcon = lblIconWiFi;
    break;
  case loadBarIconCloud:
    styleIcon = &styleIconCloud;
    objIcon = labelCloudBar;
    break;
  case loadBarIconTurtle:
    styleIcon = &styleIconTurtleStatusbar;
    objIcon = labelTurtleBar;
    break;
  default:
    Serial.println("ERROR: Undefined iconType!");
    return;
  }
  _dispDriver->esp32_lvgl_acquire();
  // set icon's color and refresh
  lv_style_set_text_color(styleIcon, lv_palette_main(LV_PALETTE_GREEN));
  lv_obj_refresh_style(objIcon, LV_PART_MAIN, LV_STYLE_PROP_ANY);
  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Builds and displays the loading screen.
*/
/**************************************************************************/
void ws_display_ui_helper::show_scr_load() {
  _dispDriver->esp32_lvgl_acquire();
  // Icon bar
  const lv_coord_t iconBarXStart =
      20; // Coordinate where the icon bar begins, on the X axis
  const lv_coord_t iconBarYOffset = 5; // Vertical offset from top of screen
  const int iconBarXSpaces = 40;       // Horizontal spaces between icons

  // add symbol_code (30px) to represent settings.json
  lblIconFile = lv_label_create(lv_scr_act());
  lv_label_set_text(lblIconFile, SYMBOL_CODE);
  // formatting
  lv_style_init(&styleIconFile);
  lv_style_set_text_color(&styleIconFile, lv_palette_main(LV_PALETTE_GREY));
  lv_style_set_text_font(&styleIconFile, &file);
  lv_obj_add_style(lblIconFile, &styleIconFile, LV_PART_MAIN);
  lv_obj_align(lblIconFile, LV_ALIGN_TOP_LEFT, iconBarXStart, iconBarYOffset);

  // add symbol_wifi (30px) to represent wifi connect
  lblIconWiFi = lv_label_create(lv_scr_act());
  lv_label_set_text(lblIconWiFi, SYMBOL_WIFI);
  lv_style_init(&styleIconWiFi);
  lv_style_set_text_color(&styleIconWiFi, lv_palette_main(LV_PALETTE_GREY));
  lv_style_set_text_font(&styleIconWiFi, &wifi_30px);
  lv_obj_add_style(lblIconWiFi, &styleIconWiFi, LV_PART_MAIN);
  lv_obj_align(lblIconWiFi, LV_ALIGN_TOP_LEFT, iconBarXStart + iconBarXSpaces,
               iconBarYOffset);

  // Add cloud
  labelCloudBar = lv_label_create(lv_scr_act());
  lv_label_set_text(labelCloudBar, SYMBOL_CLOUD);

  lv_style_init(&styleIconCloud);
  lv_style_set_text_color(&styleIconCloud, lv_palette_main(LV_PALETTE_GREY));
  lv_style_set_text_font(&styleIconCloud, &cloud_30px);
  lv_obj_add_style(labelCloudBar, &styleIconCloud, LV_PART_MAIN);
  lv_obj_align(labelCloudBar, LV_ALIGN_TOP_LEFT, 120, iconBarYOffset);

  // Add turtle
  labelTurtleBar = lv_label_create(lv_scr_act());
  lv_label_set_text(labelTurtleBar, SYMBOL_TURTLE30PX);

  lv_style_init(&styleIconTurtleStatusbar);
  lv_style_set_text_color(&styleIconTurtleStatusbar,
                          lv_palette_main(LV_PALETTE_GREY));
  lv_style_set_text_font(&styleIconTurtleStatusbar, &turtle_30px);
  lv_obj_add_style(labelTurtleBar, &styleIconTurtleStatusbar, LV_PART_MAIN);
  lv_obj_align(labelTurtleBar, LV_ALIGN_TOP_LEFT, 180, iconBarYOffset);

  // Add status text label underneath the top loading bar
  lblStatusText = lv_label_create(lv_scr_act());
  lv_label_set_long_mode(lblStatusText, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(lblStatusText, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lblStatusText, lv_color_white(), LV_PART_MAIN);
  lv_label_set_text(lblStatusText, "\0");
  lv_obj_align(lblStatusText, LV_ALIGN_TOP_MID, 0, 50);
  lv_obj_add_event_cb(lblStatusText, label_status_cb, LV_EVENT_REFRESH, NULL);

  // Add loading tooltip text label
  lblTipText = lv_label_create(lv_scr_act());
  lv_label_set_long_mode(lblTipText, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(lblTipText, &lv_font_montserrat_18, 0);
  lv_obj_set_width(lblTipText,
                   230); // TODO: This should match display width - 10px
  lv_obj_set_style_text_color(lblTipText, lv_color_white(), LV_PART_MAIN);
  lv_label_set_text(lblTipText, "\0");
  lv_obj_align(lblTipText, LV_ALIGN_BOTTOM_LEFT, 0, -40);
  timerLoadTips = lv_timer_create(lv_timer_tips_cb, 3000, NULL);

  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Deletes all objects/styles off the load screen and frees
              their resources.
*/
/**************************************************************************/
void ws_display_ui_helper::clear_scr_load() {
  _dispDriver->esp32_lvgl_acquire();
  // Delete icons
  lv_obj_del(lblStatusText);
  lv_obj_del(lblIconWiFi);
  lv_obj_del(lblIconFile);
  lv_obj_del(labelTurtleBar);
  lv_obj_del(labelCloudBar);
  // Clear all properties from styles and free all allocated memory
  lv_style_reset(&styleIconWiFi);
  lv_style_reset(&styleIconFile);
  lv_style_reset(&styleIconCloud);
  lv_style_reset(&styleIconTurtleStatusbar);
  // Stop the loading tip timer and delete the label
  remove_tip_timer();
  lv_obj_del(lblTipText);
  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Build and display an error screen.
    @param    lblError
              The generic error.
    @param    lblDesc
              Instructions or steps to resolve the error.
*/
/**************************************************************************/
void ws_display_ui_helper::show_scr_error(const char *lblError,
                                          const char *lblDesc) {
  Serial.println("ws_display_ui_helper");
  // clear the active loading screen (for now, will eventually expand to take in
  // a scr obj.)

  _dispDriver->esp32_lvgl_acquire();
  clear_scr_load();

  // Create error symbol
  labelErrorTriangle = lv_label_create(lv_scr_act());
  lv_label_set_text(labelErrorTriangle, SYMBOL_ERROR_TRIANGLE);

  lv_style_init(&styleErrorTriangle);
  lv_style_set_text_color(&styleErrorTriangle, lv_color_white());
  lv_style_set_text_font(&styleErrorTriangle, &errorTriangle);
  lv_obj_add_style(labelErrorTriangle, &styleErrorTriangle, LV_PART_MAIN);
  lv_obj_align(labelErrorTriangle, LV_ALIGN_TOP_MID, 0, 5);

  // Add error label (large)
  labelErrorHeader = lv_label_create(lv_scr_act());
  lv_label_set_text(labelErrorHeader, lblError);

  lv_style_init(&styleLabelErrorLarge);
  lv_style_set_text_color(&styleLabelErrorLarge, lv_color_white());
  lv_style_set_text_font(&styleLabelErrorLarge, &lv_font_montserrat_18);
  lv_obj_add_style(labelErrorHeader, &styleLabelErrorLarge, LV_PART_MAIN);
  lv_obj_align(labelErrorHeader, LV_ALIGN_CENTER, 0, -5);

  // Add error label (small)
  labelErrorBody = lv_label_create(lv_scr_act());
  lv_label_set_long_mode(labelErrorBody, LV_LABEL_LONG_WRAP);
  lv_label_set_text(labelErrorBody, lblDesc);

  lv_style_init(&styleLabelErrorSmall);
  lv_style_set_text_color(&styleLabelErrorSmall, lv_color_white());
  lv_style_set_text_font(&styleLabelErrorSmall, &lv_font_montserrat_12);
  lv_obj_add_style(labelErrorBody, &styleLabelErrorSmall, LV_PART_MAIN);
  // set_width used by LABEL_LONG_WRAP
  lv_obj_set_width(labelErrorBody, 220);
  lv_obj_align(labelErrorBody, LV_ALIGN_CENTER, -3, 55);

  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Build and display the monitor screen
*/
/**************************************************************************/
void ws_display_ui_helper::build_scr_monitor() {
  _dispDriver->esp32_lvgl_acquire();

  // add canvas to create a status bar
  lv_obj_t * canvas = lv_canvas_create(lv_scr_act());
  static uint8_t buffer[LV_CANVAS_BUF_SIZE_TRUE_COLOR(240, 25)];
  lv_canvas_set_buffer(canvas, buffer, 240, 25, LV_IMG_CF_TRUE_COLOR);
  lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
  lv_draw_rect_dsc_t rect_dsc;
  rect_dsc.bg_color = lv_palette_main(LV_PALETTE_GREY);
  rect_dsc.bg_opa = LV_OPA_COVER;
  lv_draw_rect_dsc_init(&rect_dsc);
  lv_canvas_draw_rect(canvas, 0, 0, 240, 25, &rect_dsc);

  // Add battery icon to status bar
  // Future TODO: Optional timer to check battery level on some boards
  // Note: FunHouse won't require this and should always be have a full battery displayed
  statusbar_icon_bat = lv_label_create(lv_scr_act());
  lv_label_set_text(statusbar_icon_bat, LV_SYMBOL_BATTERY_FULL);
  lv_obj_align(statusbar_icon_bat, LV_ALIGN_TOP_RIGHT, -5, 6);

  // Add WiFi icon to status bar
  // Future TODO: Timer to check if we are still connected to WiFi levels every 2000ms
  statusbar_icon_wifi = lv_label_create(lv_scr_act());
  lv_label_set_text(statusbar_icon_wifi, LV_SYMBOL_WIFI);
  lv_obj_align(statusbar_icon_wifi, LV_ALIGN_TOP_RIGHT, -30, 5);

  // Add Turtle icon to status bar
  lv_obj_t *labelTurtleBar = lv_label_create(lv_scr_act());
  lv_label_set_text(labelTurtleBar, SYMBOL_TURTLE30PX);
  static lv_style_t styleIconTurtleStatusbar;
  lv_style_init(&styleIconTurtleStatusbar);
  lv_style_set_text_color(&styleIconTurtleStatusbar,
                          lv_palette_main(LV_PALETTE_GREEN));
  lv_style_set_text_font(&styleIconTurtleStatusbar, &turtle_16);
  lv_obj_add_style(labelTurtleBar, &styleIconTurtleStatusbar,
                   LV_PART_MAIN);
  lv_obj_align(labelTurtleBar, LV_ALIGN_TOP_LEFT, 5, 5);

  // Add a label to hold console text
  // TODO: Still having some overlap between the top console text and the
  // status bar.. this should be fixed in the sim. first before release
  terminalLabel = lv_label_create(lv_scr_act());
  lv_obj_align(terminalLabel, LV_ALIGN_BOTTOM_LEFT, 3, 0);
  lv_obj_set_width(terminalLabel, 230);
  lv_label_set_long_mode(terminalLabel, LV_LABEL_LONG_WRAP);
  lv_style_init(&styleTerminalLabel);
  lv_style_set_text_color(&styleTerminalLabel, lv_color_white());
  lv_obj_add_style(terminalLabel, &styleTerminalLabel, LV_PART_MAIN);
  lv_label_set_text_static(terminalLabel, terminalTextBuffer);
  lv_obj_move_background(terminalLabel);

  Serial.println("main app. screen built!");

  _dispDriver->esp32_lvgl_release();
}

/**************************************************************************/
/*!
    @brief    Add text on the terminal label and displays it.
    @param    text
              Text to display on the terminal, should end in "\n"
*/
/**************************************************************************/
void ws_display_ui_helper::add_text_to_terminal(const char *text) {
  Serial.println("add_text_to_terminal");
  char txtBuffer[256]; // temporary text buffer for snprintf
  snprintf(txtBuffer, 256, text);
  addToTerminal(txtBuffer);
}

/**************************************************************************/
/*!
    @brief    Adds a line of text on the terminal label and displays it.
    @param    text
              A line of text to display on the terminal.
    @note     Reference:
   https://github.com/lvgl/lv_demos/blob/release/v6/lv_apps/terminal/terminal.c
*/
/**************************************************************************/
void ws_display_ui_helper::addToTerminal(const char * txt_in)
{
    // Calculate text size
    size_t txt_len = strlen(txt_in);
    size_t old_len = strlen(terminalTextBuffer);

    // If the data is longer then the terminal ax size show the last part of data
    if(txt_len > MAX_CONSOLE_TEXT_LEN) {
        txt_in += (txt_len - MAX_CONSOLE_TEXT_LEN);
        txt_len = MAX_CONSOLE_TEXT_LEN;
        old_len = 0;
    }

    // If the text become too long 'forget' the oldest lines
    else if(old_len + txt_len > MAX_CONSOLE_TEXT_LEN) {
        uint16_t new_start;
        for(new_start = 0; new_start < old_len; new_start++) {
            if(terminalTextBuffer[new_start] == '\n') {
                if(new_start >= txt_len) {
                    while(terminalTextBuffer[new_start] == '\n' || terminalTextBuffer[new_start] == '\r') new_start++;
                    break;
                }
            }
        }

        // If it wasn't able to make enough space on line breaks simply forget the oldest characters
        if(new_start == old_len) {
            new_start = old_len - (MAX_CONSOLE_TEXT_LEN - txt_len);
        }

        // Move the remaining text to the beginning
        uint16_t j;
        for(j = new_start; j < old_len; j++) {
            terminalTextBuffer[j - new_start] = terminalTextBuffer[j];
        }
        old_len = old_len - new_start;
        terminalTextBuffer[old_len] = '\0';

    }

    // Copy new text to the text buffer
    memcpy(&terminalTextBuffer[old_len], txt_in, txt_len);
    terminalTextBuffer[old_len + txt_len] = '\0';

    // Update label
    _dispDriver->esp32_lvgl_acquire();
    lv_label_set_text_static(terminalLabel, terminalTextBuffer);
    _dispDriver->esp32_lvgl_release();
}
