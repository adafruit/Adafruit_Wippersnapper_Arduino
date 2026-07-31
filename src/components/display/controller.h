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

#define MAX_DISPLAYS 4         ///< Maximum number of displays
#define MAX_CANVAS_CHUNKS 1024 ///< Sanity cap on chunks per reassembled canvas
#define MAX_FEED_HISTORY_STORAGE 512
#define MAX_CANVAS_SIZE                                                        \
  (MAX_FEED_HISTORY_STORAGE *                                                  \
   1024) ///< Sanity cap on the reserve() for a reassembled BMP
#define CANVAS_STREAM_IDLE_MS                                                  \
  10000 ///< How long after the last canvas activity (a display Add, or an
        ///< ingested chunk) IsCanvasStreaming() keeps reporting true. Bounds
        ///< the cost of the long per-packet MQTT idle timeout: that timeout is
        ///< paid on every empty read, so leaving it armed for the whole
        ///< CANVAS_ASSEMBLY_TIMEOUT_MS window held the main loop near 2Hz for
        ///< up to five minutes when the canvas never arrived. Sized well above
        ///< the inter-chunk gap of a healthy stream so an in-flight canvas
        ///< never falls back to the short poll mid-assembly.
#define CANVAS_ASSEMBLY_TIMEOUT_MS                                             \
  300000 ///< While a display is awaiting its canvas, how long to hold
         ///< loopSleep() awake for a complete assembly before giving up. Sized
         ///< to cover the broker's re-transmit window (5 attempts, one per
         ///< keepalive). On expiry the await is abandoned so loopSleep() can
         ///< reach its run-duration deadline and sleep, rather than staying
         ///< awake forever on battery.

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
  void update(int32_t rssi, bool is_connected);

  // Error handling
  void PublishDisplayComponentError(ws_display_InterfaceDescriptor iface,
                                    const char *error);
  int8_t resolveDisplayOrPublishError(ws_display_Write *msg,
                                      const char *errorMessage);

  bool IsWriteInProgress() { return _write_in_progress; }
  bool IsAwaitingWrite() { return _awaiting_write; }
  bool HasPendingWriteComplete() { return _write_complete_pending; }
  /*!
      @brief  True when canvas chunks are expected or mid-reassembly, so the
              MQTT read loop should favour a long per-packet idle timeout over a
              responsive poll.
              Gated on recent canvas activity as well as the write flags: the
              long timeout is paid on every empty read, so it must only stay
              armed while the stream is plausibly live. Without the
              CANVAS_STREAM_IDLE_MS bound, an Add whose canvas never arrived
              held the timeout armed for the whole CANVAS_ASSEMBLY_TIMEOUT_MS
              window, dragging the main loop (and every component update() it
              drives) down to ~2Hz for up to five minutes.
      @return True if a canvas write is in flight, False otherwise.
  */
  bool IsCanvasStreaming() {
    if (!_awaiting_write && !_write_in_progress)
      return false;
    return (millis() - _last_canvas_activity_ms) < CANVAS_STREAM_IDLE_MS;
  }
  void handleCanvasAssemblyTimeout();
  bool publishPendingWriteComplete();

private:
  DisplayHardware *_displays[MAX_DISPLAYS] = {nullptr};
  uint8_t _num_displays;
  unsigned long _last_bar_update;

  // General display management
  int8_t findDisplayIndexByName(const char *name);
  bool removeExistingDisplayByName(const char *name);

  // Display write handling
  bool handleTextWrite(ws_display_Write *msg);
  bool handleCanvasWrite(ws_display_Write *msg);
  bool ingestCanvasChunk(ws_display_Write *msg);
  static bool cbDecodeCanvasChunk(pb_istream_t *stream, const pb_field_t *field,
                                  void **arg);
  bool drawCanvasToDisplay(ws_display_Write *msg);
  void resetCanvasReassembly();
  uint32_t
      _current_canvas_checksum; ///< id of canvas being reassembled (0 = none)
  uint32_t _canvas_chunk_total; ///< expected total chunk count
  uint32_t _canvas_chunks_received; ///< distinct slots filled so far
  uint32_t _canvas_total_size; ///< expected assembled bitmap size, in bytes
  std::vector<std::vector<uint8_t>>
      _canvas_chunks; ///< per-chunk byte regions, indexed by chunk_id - 1
  std::vector<uint8_t>
      _pending_chunk; ///< bytes captured by the most recent decode callback
  std::vector<uint8_t>
      _bmp; ///< fully reassembled bitmap, built from _canvas_chunks
  bool _write_in_progress; ///< True if a write is currently being processed
                           ///< (gates loopSleep() sleep entry)
  bool _write_complete_pending; ///< True if a WriteComplete D2B is queued to
                                ///< publish from loop() (deferred out of the
                                ///< receive callback)
  bool _awaiting_write; ///< True from when a display is added until it draws a
                        ///< write this wake; keeps loopSleep() awake until the
                        ///< canvas is fully assembled (WriteComplete)
  unsigned long
      _awaiting_write_since_ms; ///< millis() when the current await window
                                ///< began (set once, at Add); drives the
                                ///< CANVAS_ASSEMBLY_TIMEOUT_MS log
  unsigned long
      _last_canvas_activity_ms; ///< millis() of the most recent canvas activity
                                ///< (a display Add, or an ingested chunk);
                                ///< re-armed per chunk so IsCanvasStreaming()
                                ///< tracks a live stream rather than the whole
                                ///< await window
};
extern wippersnapper Ws; ///< Global V2 instance
#endif                   // WS_DISPLAY_CONTROLLER_H
