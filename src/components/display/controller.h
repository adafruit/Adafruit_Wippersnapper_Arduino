/*!
 * @file src/components/display/controller.h
 *
 * Controller for the display API (V2)
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISPLAY_CONTROLLER_H
#define WS_DISPLAY_CONTROLLER_H
#include "hardware.h"
#include "wippersnapper.h"

#define MAX_DISPLAYS 4 ///< Maximum number of displays

class wippersnapper;   ///< Forward declaration
class DisplayHardware; ///< Forward declaration

/*!
    @brief  Routes messages using the display.proto API to the
            appropriate hardware classes, controls and tracks
            the state of displays.
*/
class DisplayController {
public:
  DisplayController();
  ~DisplayController();
  bool Router(pb_istream_t *stream);
  /*!
      @brief  Handles a request to add a display.
      @param  msg   The Display Add message.
      @param  name  The unique name for the new display.
      @return True if the display was added successfully, False otherwise.
  */
  bool Handle_Display_Add(ws_display_Add *msg, const char *name);
  bool removeExistingDisplayByName(const char *name);
  /*!
      @brief  Handles a request to remove a display.
      @param  name  The unique name of the display to remove.
      @return True if the display was removed, False otherwise.
  */
  bool Handle_Display_Remove(const char *name);
  /*!
      @brief  Handles a request to write to a display.
      @param  msg   The Display Write message.
      @param  name  The unique name of the target display.
      @return True if the write succeeded, False otherwise.
  */
  bool Handle_Display_Write(ws_display_Write *msg, const char *name);
  void update(int32_t rssi, bool is_connected);

private:
  DisplayHardware *_displays[MAX_DISPLAYS] = {nullptr};
  uint8_t _num_displays;
  unsigned long _last_bar_update; ///< Timestamp of last status bar update
  int8_t findDisplayByName(const char *name);
};
extern wippersnapper Ws; ///< Global V2 instance
#endif                   // WS_DISPLAY_CONTROLLER_H
