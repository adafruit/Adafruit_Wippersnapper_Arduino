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
#include <vector>

#define MAX_DISPLAYS 4        ///< Maximum number of displays
#define MAX_CANVAS_CHUNKS 256 ///< Sanity cap on chunks per reassembled canvas

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
  bool Handle_Display_Add(ws_display_Add *msg);
  bool Handle_Display_Remove(ws_display_Remove *msg);
  bool Handle_Display_Write(ws_display_Write *msg);
  void PublishDisplayComponentError(ws_display_InterfaceDescriptor iface,
                                    const char *error);
  void update(int32_t rssi, bool is_connected);

private:
  DisplayHardware *_displays[MAX_DISPLAYS] = {nullptr};
  uint8_t _num_displays;
  unsigned long _last_bar_update; ///< Timestamp of last status bar update
  bool removeExistingDisplayByName(const char *name);
  int8_t findDisplayIndexByName(const char *name);

  // --- Canvas chunk reassembly (single in-flight canvas) ---
  static bool cbDecodeCanvasChunk(pb_istream_t *stream,
                                  const pb_field_t *field, void **arg);
  void resetCanvasReassembly(); ///< Frees regions + pending, zeroes state

  uint32_t _canvas_id;              ///< id of canvas being reassembled (0 = none)
  uint32_t _canvas_chunk_total;     ///< expected total chunk count
  uint32_t _canvas_chunks_received; ///< distinct slots filled so far
  uint32_t _canvas_total_size;      ///< expected assembled bitmap size, in bytes
  std::vector<std::vector<uint8_t>>
      _canvas_chunks; ///< per-chunk byte regions, indexed by chunk_id - 1
  std::vector<uint8_t>
      _pending_chunk; ///< bytes captured by the most recent decode callback
};
extern wippersnapper Ws; ///< Global V2 instance
#endif                   // WS_DISPLAY_CONTROLLER_H
