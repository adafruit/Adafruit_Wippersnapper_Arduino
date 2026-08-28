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

#define MAX_DISPLAYS                                                           \
  4 ///< Maximum number of displays, NOTE: There can only be ONE marquee-capable
    ///< display at a time
#define MAX_CANVAS_CHUNKS 1024 ///< Sanity cap on chunks per reassembled canvas
#define MAX_CANVAS_DRAW_ATTEMPTS                                               \
  3 ///< Maximum number of attempts to draw a reassembled canvas to a display
#define MAX_FEED_HISTORY_STORAGE                                               \
  512 ///< Total number of bytes that an Adafruit IO feed (with NO history) can
      ///< store
#define MAX_CANVAS_SIZE                                                        \
  (MAX_FEED_HISTORY_STORAGE *                                                  \
   1024) ///< Sanity cap on the reserve() for a reassembled BMP
#define CANVAS_STREAM_IDLE_MS                                                  \
  10000 ///< How long after the last canvas activity (a display Add, or an
        ///< ingested chunk) isImageStreaming() keeps reporting true. Bounds
        ///< the cost of the long per-packet MQTT idle timeout: that timeout is
        ///< paid on every empty read, so leaving it armed for the whole time a
        ///< canvas is outstanding held the main loop near 2Hz when the canvas
        ///< never arrived. Sized well above the inter-chunk gap of a healthy
        ///< stream so an in-flight canvas never falls back to the short poll
        ///< mid-assembly.

class wippersnapper;   ///< Forward declaration
class DisplayHardware; ///< Forward declaration

/*!
    @brief  Tracks the marquee canvas lifecycle for the controller as a whole.
            Any state other than IDLE means marquee work is outstanding and
            loopSleep() must keep the device awake.
*/
enum class marquee_state_t {
  IDLE,      ///< No marquee work outstanding
  AWAITING,  ///< Display added; the broker's canvas write has not started
  STREAMING, ///< Canvas chunks are arriving
  DRAWING    ///< All chunks in; reassembling and drawing to the panel
};

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
  void update(int32_t rssi, bool is_connected);
  bool UpdateComplete();
  void ResetFlags();

  // Error handling
  void PublishDisplayComponentError(ws_display_InterfaceDescriptor iface,
                                    const char *error);
  int8_t resolveDisplayOrPublishError(ws_display_Write *msg,
                                      const char *errorMessage);

  // Marquee helpers
  bool isImageStreaming();

private:
  DisplayHardware *_displays[MAX_DISPLAYS] = {
      nullptr}; ///< Display hardware instances
  uint8_t
      _num_displays; ///< Number of displays currently managed by the controller
  unsigned long
      _last_bar_update; ///< Last time the status bar was updated, in millis()
  marquee_state_t _marquee_state; ///< Tracks the marquee canvas's lifecycle for
                                  ///< the controller

  // General display management
  int8_t findDisplayIndexByName(const char *name);
  bool removeExistingDisplayByName(const char *name);

  // Display Write Handling
  bool handleTextWrite(ws_display_Write *msg);
  bool handleImageWrite(ws_display_Write *msg);
  bool processImageChunk(ws_display_Write *msg);
  static bool cbDecodeImageChunk(pb_istream_t *stream, const pb_field_t *field,
                                 void **arg);
  bool drawImage(ws_display_Write *msg);
  bool publishWriteComplete();
  void resetImage();
  uint32_t _canvas_checksum;   ///< id of canvas being reassembled (0 = none)
  uint32_t _image_chunk_total; ///< expected total chunk count
  uint32_t _image_chunks_received; ///< distinct slots filled so far
  size_t _image_chunk_bytes;       ///< running sum of every filed chunk's size,
                                   ///< in bytes; used to reserve _bmp exactly
  uint32_t _expected_image_size;   ///< expected assembled bitmap size, in bytes
  // TODO Tuesday: Do we need the std:: nomenclature here?
  std::vector<std::vector<uint8_t>>
      _image_chunks; ///< Holds the reconstructed canvas chunks for reassembly
                     ///< into a full bitmap
  std::vector<uint8_t>
      _pending_chunk; ///< bytes captured by the most recent decode callback
  std::vector<uint8_t>
      _bmp; ///< fully reassembled bitmap, built from _image_chunks
  unsigned long
      _prv_image_activity; ///< millis() of the most recent canvas activity
                           ///< (a display Add, or an ingested chunk);
                           ///< re-armed per chunk so isImageStreaming()
                           ///< tracks a live stream rather than the whole
                           ///< time a canvas is outstanding
};
extern wippersnapper Ws; ///< Global V2 instance
#endif                   // WS_DISPLAY_CONTROLLER_H
