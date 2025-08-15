/*!
 * @file src/components/display/controller.h
 *
 * Controller for the display API
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
#ifndef WS_DISPLAY_CONTROLLER_H
#define WS_DISPLAY_CONTROLLER_H
#include "Wippersnapper.h"
#include "hardware.h"

class Wippersnapper_V2; ///< Forward declaration
class DisplayHardware;  ///< Forward declaration

/**************************************************************************/
/*!
    @brief  Routes messages using the display.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of displays.
*/
/**************************************************************************/
class DisplayController {
public:
  DisplayController();
  ~DisplayController();
  bool Handle_Display_AddOrReplace(wippersnapper_display_v1_DisplayAddOrReplace *msgAdd);
  //bool Handle_Display_Write(pb_istream_t *stream);
  //bool Handle_Display_Remove(pb_istream_t *stream);
private:
  //DisplayHardware *_display_strands[MAX_DISPLAY_STRANDS] = {
};
extern Wippersnapper Ws; ///< Global WS instance
#endif