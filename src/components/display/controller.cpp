/*!
 * @file src/components/display/controller.cpp
 *
 * Implementation for the display API controller (V2).
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
#include "controller.h"

DisplayController::DisplayController() {
  _num_displays = 0;
  _last_bar_update = 0;
  _current_canvas_checksum = 0;
  _canvas_chunk_total = 0;
  _canvas_chunks_received = 0;
  _canvas_total_size = 0;
}

DisplayController::~DisplayController() {
  for (int i = 0; i < _num_displays; i++) {
    delete _displays[i];
  }
  _num_displays = 0;
  resetCanvasReassembly();
}

/*!
    @brief  Frees all buffered canvas chunk regions and the assembled bitmap,
            and zeroes the per-canvas reassembly state. Does NOT touch
            _pending_chunk: that holds the current message's just-decoded bytes
            and is owned by the decode callback / Handle_Display_Write, which
            may call this mid-message (e.g. on a new image id) before filing
            those bytes.
*/
void DisplayController::resetCanvasReassembly() {
  // swap-with-empty releases the heap capacity, not just the size
  std::vector<std::vector<uint8_t>>().swap(_canvas_chunks);
  std::vector<uint8_t>().swap(_bmp);
  _current_canvas_checksum = 0;
  _canvas_chunk_total = 0;
  _canvas_chunks_received = 0;
  _canvas_total_size = 0;
}

/*!
    @brief  Routes messages using the display.proto API to the
            appropriate controller functions.
    @param  stream  The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool DisplayController::Router(pb_istream_t *stream) {
  WS_DEBUG_PRINTLN("=> Routing Display message...");
  // Save the stream before decoding — the write case re-decodes with a
  // chunk_data callback wired up, and ws_pb_decode consumes the stream.
  pb_istream_t saved_stream = *stream;
  WS_DEBUG_PRINTLN("=> Saved stream for re-decode if needed...");

  // ws_display_B2D is large (~1.4KB — its union embeds Write.message[1024]).
  // Router runs nested inside the outer signal pb_decode (via the cb_payload
  // callback), so keeping this struct on the stack risks overflowing the
  // loopTask stack. Heap-allocate it and free it before every return.
  ws_display_B2D *b2d = new ws_display_B2D;
  *b2d = ws_display_B2D_init_zero;
  WS_DEBUG_PRINTLN("=> Decoding Display B2D envelope...");
  if (!ws_pb_decode(stream, ws_display_B2D_fields, b2d)) {
    WS_DEBUG_PRINTLN("[display] ERROR: Unable to decode Display B2D envelope");
    delete b2d;
    return false;
  }

  WS_DEBUG_PRINT("[display] Decoded B2D envelope with payload tag: ");
  WS_DEBUG_PRINTLNVAR(b2d->which_payload);

  bool status = false;
  switch (b2d->which_payload) {
  case ws_display_B2D_add_tag:
    WS_DEBUG_PRINTLN("[display] Routing Display Add message...");
    status = Handle_Display_Add(&b2d->payload.add);
    break;
  case ws_display_B2D_remove_tag:
    status = Handle_Display_Remove(&b2d->payload.remove);
    break;
  case ws_display_B2D_write_tag: {
    // Re-decode from the saved stream with the Canvas chunk_data callback
    // wired up; the first decode skipped chunk_data (no callback set).
    //
    // chunk_data lives inside the `write` oneof member. nanopb memsets a oneof
    // submessage to zero the first time it sets which_payload (pb_decode.c
    // ~L528), which would wipe our pre-set callback. Pre-setting which_payload
    // AND decoding with PB_DECODE_NOINIT skips both the default-init (which
    // would otherwise reset which_payload to 0) and that memset, so the
    // callback survives. init_zero gives a clean struct since we skip init.
    ws_display_B2D *b2d_w = new ws_display_B2D;
    *b2d_w = ws_display_B2D_init_zero;
    b2d_w->which_payload = ws_display_B2D_write_tag;
    b2d_w->payload.write.image.chunk_data.funcs.decode = cbDecodeCanvasChunk;
    b2d_w->payload.write.image.chunk_data.arg = this;
    if (!pb_decode_ex(&saved_stream, ws_display_B2D_fields, b2d_w,
                      PB_DECODE_NOINIT)) {
      WS_DEBUG_PRINT("[display] ERROR: Failed to re-decode write w/ canvas: ");
      WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(&saved_stream));
      delete b2d_w;
      delete b2d;
      return false;
    }
    status = Handle_Display_Write(&b2d_w->payload.write);
    delete b2d_w;
    break;
  }
  default:
    WS_DEBUG_PRINTLN("[display] WARNING: Unsupported Display payload");
    break;
  }

  delete b2d;
  return status;
}

/*!
    @brief  Resolves component-name based driver/mode defaults for EPD displays.
            Called before passing the Add message to hardware so that the
            controller owns the "what driver" decision and hardware just inits.
    @param  msg  The Display Add message (may be modified in place).
    @param  name  The unique name for the display to add or replace.
    @return True if defaults were resolved, False otherwise.
*/
static bool resolveEpdDefaults(ws_display_Add *msg, const char *name) {
  if (!msg->has_interface_type ||
      msg->interface_type.which_descriptor !=
          ws_display_InterfaceDescriptor_spi_epd_tag)
    return false;
  if (msg->which_config != ws_display_Add_config_epd_tag)
    return false;

  ws_display_EPDConfig *config = &msg->config.config_epd;

  // MagTag auto-detection is handled at hardware level (needs SPI probing),
  // but we can still set mode default
  if (strcmp(name, "eink-magtag") == 0) {
    if (config->mode == ws_display_EPDMode_EPD_MODE_UNSPECIFIED)
      config->mode = ws_display_EPDMode_EPD_MODE_GRAYSCALE4;
    return true;
  }

  // Map specific component names to driver + default mode
  struct EpdMapping {
    const char *component;
    const char *driver;
    ws_display_EPDMode mode;
  } mappings[] = {
      {"eink-29-flexible-monochrome-296x128", "UC8151",
       ws_display_EPDMode_EPD_MODE_MONO},
      {"eink-37-monochrome-416x240", "UC8253",
       ws_display_EPDMode_EPD_MODE_MONO},
      {"eink-42-grayscale-300x400", "SSD1683",
       ws_display_EPDMode_EPD_MODE_GRAYSCALE4},
      {"eink-583-monochrome-648x480", "UC8179",
       ws_display_EPDMode_EPD_MODE_MONO},
  };

  for (const EpdMapping &m : mappings) {
    if (strcmp(name, m.component) == 0) {
      strncpy(msg->driver, m.driver, sizeof(msg->driver) - 1);
      msg->driver[sizeof(msg->driver) - 1] = '\0';
      if (config->mode == ws_display_EPDMode_EPD_MODE_UNSPECIFIED)
        config->mode = m.mode;
      WS_DEBUG_PRINT("[display] Resolved component '");
      WS_DEBUG_PRINTVAR(name);
      WS_DEBUG_PRINT("' -> driver '");
      WS_DEBUG_PRINTVAR(msg->driver);
      WS_DEBUG_PRINTLN("'");
      return true;
    }
  }

  WS_DEBUG_PRINT("[display] No specific driver/mode defaults for component '");
  WS_DEBUG_PRINTVAR(name);
  WS_DEBUG_PRINTLN("'");
  return false;
}

/*!
    @brief  Resolves component-name based driver/panel defaults for RGB666
            dotclock displays (Qualia).
    @param  msg  The Display Add message (may be modified in place).
    @param  name  The unique name for the display to add or replace.
    @return True if defaults were resolved, False otherwise.
*/
static bool resolveRgb666Defaults(ws_display_Add *msg, const char *name) {
  if (!msg->has_interface_type ||
      msg->interface_type.which_descriptor !=
          ws_display_InterfaceDescriptor_ttl_rgb666_tag)
    return false;
  if (msg->which_config != ws_display_Add_config_display_tag)
    return false;

  struct Rgb666DefaultMapping {
    const char *component;
    const char *driver;
    const char *panel;
  } mappings[] = {
      {"qualia-round-21-480x480", "ST7701S", "TL021WVC02"},
      {"qualia-bar-32-320x820", "ST7701S", "TL032FWV01"},
  };

  for (const Rgb666DefaultMapping &m : mappings) {
    if (strcmp(name, m.component) == 0) {
      strncpy(msg->driver, m.driver, sizeof(msg->driver) - 1);
      msg->driver[sizeof(msg->driver) - 1] = '\0';
      strncpy(msg->panel, m.panel, sizeof(msg->panel) - 1);
      msg->panel[sizeof(msg->panel) - 1] = '\0';
      WS_DEBUG_PRINT("[display] Resolved component '");
      WS_DEBUG_PRINTVAR(name);
      WS_DEBUG_PRINT("' -> driver '");
      WS_DEBUG_PRINTVAR(msg->driver);
      WS_DEBUG_PRINT("', panel '");
      WS_DEBUG_PRINTVAR(msg->panel);
      WS_DEBUG_PRINTLN("'");
      return true;
    }
  }
  WS_DEBUG_PRINT("[display] No specific driver/panel defaults for component '");
  WS_DEBUG_PRINTVAR(name);
  WS_DEBUG_PRINTLN("'");
  return false;
}

/*!
    @brief  Handles a request to add or replace a display.
    @param  msg  The Display Add message.
    @return True if successful, False otherwise.
*/
bool DisplayController::Handle_Display_Add(ws_display_Add *msg) {
  if (msg == nullptr || msg->name == nullptr) {
    WS_DEBUG_PRINTLN("[display] ERROR: Display name is null!");
    return false;
  }

  WS_DEBUG_PRINT("[display] Adding display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  bool foundDefaults = false;
  // Resolve component-name defaults before passing to hardware
  if (msg->type == ws_display_DisplayClass_DISPLAY_CLASS_EPD) {
    foundDefaults = resolveEpdDefaults(msg, msg->name);
  } else if (msg->type == ws_display_DisplayClass_DISPLAY_CLASS_TFT &&
             (msg->has_interface_type &&
              msg->interface_type.which_descriptor ==
                  ws_display_InterfaceDescriptor_ttl_rgb666_tag)) {
    foundDefaults = resolveRgb666Defaults(msg, msg->name);
  }
  if (!foundDefaults) {
    // The resolve defaults funcs are not mandatory (come in proto)
    WS_DEBUG_PRINTLN("[display] No defaults resolved for this display");
  }

  // If display with same name exists, remove it first to allow replacement
  if (removeExistingDisplayByName(msg->name)) {
    WS_DEBUG_PRINT("[display] Replaced existing display with same name '");
    WS_DEBUG_PRINTVAR(msg->name);
    WS_DEBUG_PRINTLN("'");
  }

  if (_num_displays >= MAX_DISPLAYS) {
    WS_DEBUG_PRINTLN("[display] ERROR: Maximum number of displays reached!");
    PublishDisplayComponentError(
        msg->interface_type,
        "Failed to add. Maximum number of displays reached");
    return false;
  }

  // Create and initialize new display hardware
  DisplayHardware *hw = new DisplayHardware();
  if (!hw->begin(msg)) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to initialize display hardware!");
    delete hw;
    PublishDisplayComponentError(
        msg->interface_type,
        "Failed to initialize display hardware for add request");
    return false;
  }

  // Show splash screen and status bar
  hw->initialise(Ws._configV2.aio_user);

  _displays[_num_displays] = hw;
  _num_displays++;

  // Handle optional initial write
  if (msg->has_write) {
    WS_DEBUG_PRINTLN("[display] Processing initial write...");
    // TODO: Unlike Router()'s write path, this call does NOT wire the
    // chunk_data decode callback (cbDecodeCanvasChunk), so _pending_chunk is
    // empty here. An Add-bundled image write therefore fails handleCanvasWrite's
    // empty-chunk check; only text writes are supported on this path today.
    // TODO: To support Add-bundled image writes, replicate Router()'s
    // pb_decode_ex + PB_DECODE_NOINIT callback wiring for msg->write.image.
    Handle_Display_Write(&msg->write);
  }

  WS_DEBUG_PRINT("[display] Display added successfully: ");
  WS_DEBUG_PRINTLNVAR(msg->name);
  return true;
}

/*!
    @brief  Publishes a component error related to a display add request.
    @param  iface  The Interface descriptor for the display that caused the
   error.
    @param  error The error message to publish.
*/
void DisplayController::PublishDisplayComponentError(
    ws_display_InterfaceDescriptor iface, const char *error) {
  switch (iface.which_descriptor) {
  case ws_display_InterfaceDescriptor_spi_epd_tag:
    Ws.error_handler->publishComponentError(iface.descriptor.spi_epd.spi,
                                            error);
    break;
  case ws_display_InterfaceDescriptor_spi_tft_tag:
    Ws.error_handler->publishComponentError(iface.descriptor.spi_tft.spi,
                                            error);
    break;
  case ws_display_InterfaceDescriptor_i2c_tag:
    Ws.error_handler->publishComponentError(iface.descriptor.i2c, error);
    break;
  case ws_display_InterfaceDescriptor_ttl_rgb666_tag:
    Ws.error_handler->publishComponentError(iface.descriptor.ttl_rgb666.pin_b0,
                                            error);
    break;
  case ws_display_InterfaceDescriptor_i8080_tag:
    Ws.error_handler->publishComponentError(iface.descriptor.i8080.pin_d0,
                                            error);
    break;
  case ws_display_InterfaceDescriptor_dsi_tag:
    Ws.error_handler->publishComponentError(iface.descriptor.dsi.pin_rst,
                                            error);
    break;
  default:
    WS_DEBUG_PRINT("[display] WARNING: Unknown display interface");
    WS_DEBUG_PRINTVAR(iface.which_descriptor);
    Ws.error_handler->publishComponentError("Unknown interface", error);
    break;
  }
}

/*!
    @brief  Removes an existing display with the same name as the new one being
   added. This ensures that adding a display with a duplicate name will replace
            the old one instead of creating a conflict.
    @param  name  The name of the display to remove.
    @return True if a display was removed, False if no existing display had the
   same name.
*/
bool DisplayController::removeExistingDisplayByName(const char *name) {
  if (!name) {
    WS_DEBUG_PRINTLN("[display] ERROR: Null display name provided for removal");
    return false;
  }

  // If display with same name exists, remove it first
  int8_t existingIdx = findDisplayIndexByName(name);
  if (existingIdx >= 0) {
    WS_DEBUG_PRINTLN("[display] Replacing existing display");
    delete _displays[existingIdx];
    // Shift remaining displays down
    for (int i = existingIdx; i < _num_displays - 1; i++) {
      _displays[i] = _displays[i + 1];
    }
    _displays[_num_displays - 1] = nullptr;
    _num_displays--;
    return true;
  }
  return false;
}

/*!
    @brief  Handles a request to remove a display.
    @param  msg  The Display Remove proto message identifying the display.
    @return True if successful, False otherwise.
*/
bool DisplayController::Handle_Display_Remove(ws_display_Remove *msg) {
  WS_DEBUG_PRINT("[display] Removing display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);
  bool did_remove = removeExistingDisplayByName(msg->name);
  if (!did_remove) {
    const char *error_msg = "Display not found for remove request";
    WS_DEBUG_PRINTLN("[display] WARNING: Display not found");
    if (msg->has_descriptor) {
      PublishDisplayComponentError(msg->descriptor, error_msg);
    } else {
      WS_DEBUG_PRINTLN("[display] WARNING: No descriptor in remove message to "
                       "publish error against");
    }
  }
  return did_remove;
}

/*!
    @brief  nanopb decode callback for Canvas.chunk_data. Captures one chunk's
            raw bytes into the controller's pending-chunk holder; placement and
            reassembly happen in Handle_Display_Write.
    @param  stream  The nanopb input stream positioned at the chunk bytes.
    @param  field   The chunk_data field descriptor (unused).
    @param  arg     Pointer to the owning DisplayController (set in Router).
    @return True on success, False on allocation/read failure.
*/
bool DisplayController::cbDecodeCanvasChunk(pb_istream_t *stream,
                                            const pb_field_t *field,
                                            void **arg) {
  (void)field;
  DisplayController *self = (DisplayController *)*arg;
  size_t len = stream->bytes_left;

  // Drop any uncommitted pending bytes (e.g. malformed prior message)
  self->_pending_chunk.clear();
  if (len == 0)
    return true;

  self->_pending_chunk.resize(len);
  if (!pb_read(stream, (pb_byte_t *)self->_pending_chunk.data(), len)) {
    self->_pending_chunk.clear();
    return false;
  }
  return true;
}

/*!
    @brief  Handles a request to write to a display.
    @param  msg  The Display Write message.
    @return True if successful, False otherwise.
*/
bool DisplayController::Handle_Display_Write(ws_display_Write *msg) {
  if (!msg || !msg->name) {
    WS_DEBUG_PRINTLN("[display] ERROR: Invalid display write request!");
    return false;
  }

  // Display Write can be either a text message or an "image" ("marquee")
  if (msg->has_image) {
    return handleCanvasWrite(msg);
  } else {
    return handleTextWrite(msg);
  }
}

/*!
    @brief  Renders a monospace text message to the target display.
    @param  msg  The Display Write message (name + message text).
    @return True if written, False otherwise.
*/
bool DisplayController::handleTextWrite(ws_display_Write *msg) {
  WS_DEBUG_PRINT("[display] Writing to display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  int8_t idx = resolveDisplayOrPublishError(msg, "Display not found for write request");
  if (idx < 0) {
    return false;
  }

  return _displays[idx]->write(msg);
}

/*!
    @brief  Ingests one canvas chunk and, once every chunk has arrived,
            reassembles the full BMP and draws it to the target display.
    @param  msg  The Display Write message carrying the image chunk.
    @return True if the chunk was accepted (or the canvas drawn), False on
            error.
*/
bool DisplayController::handleCanvasWrite(ws_display_Write *msg) {
  // Ingest this chunk of data, if we're not done ingesting the full canvas yet.
  if (!ingestCanvasChunk(msg)) {
    return false;
  }

  // Wait for more chunks before assembling.
  if (_canvas_chunks_received < _canvas_chunk_total)
    return true;

  // All chunks received: assemble the full BMP and draw it to the display.
  return drawCanvasToDisplay(msg);
}

/*!
    @brief  Validates and files a single canvas chunk into the reassembly
            buffer, tracking how many distinct chunks have arrived.
    @param  msg  The Display Write message carrying the image chunk.
    @return False on validation failure (reassembly state is reset before
            returning), True if the chunk was accepted.
*/
bool DisplayController::ingestCanvasChunk(ws_display_Write *msg) {
  // Did the decode callback actually decode any bytes into the chunk?
  if (_pending_chunk.empty()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas chunk_data missing/empty");
    resetCanvasReassembly();
    return false;
  }

  // Split out the chunk metadata
  uint32_t canvas_checksum = msg->image.checksum;
  uint32_t canvas_total_size = msg->image.size;
  uint32_t canvas_chunk_id = msg->image.chunk_id;
  uint32_t canvas_chunk_total = msg->image.chunk_total;
  WS_DEBUG_PRINT("[display] Canvas Checksum: ");
  WS_DEBUG_PRINTVAR(canvas_checksum);
  WS_DEBUG_PRINT(", Total Size: ");
  WS_DEBUG_PRINTVAR(canvas_total_size);
  WS_DEBUG_PRINT(", Chunk ID: ");
  WS_DEBUG_PRINTVAR(canvas_chunk_id);
  WS_DEBUG_PRINT(", Chunk Total: ");
  WS_DEBUG_PRINTVAR(canvas_chunk_total);
  WS_DEBUG_PRINT(", Chunk Bytes: ");
  WS_DEBUG_PRINTLNVAR((uint32_t)_pending_chunk.size());


  // Validate the chunk metadata
  if (canvas_chunk_total == 0 || canvas_chunk_total > MAX_CANVAS_CHUNKS ||
      canvas_chunk_id < 1 || canvas_chunk_id > canvas_chunk_total) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas chunk id/total out of range");
    resetCanvasReassembly();
    return false;
  }

  // New incoming image? Reset the reassembly state and start over.
  if (canvas_checksum != _current_canvas_checksum) {
    resetCanvasReassembly();
    _current_canvas_checksum = canvas_checksum;
    _canvas_chunk_total = canvas_chunk_total;
    _canvas_total_size = canvas_total_size;
    _canvas_chunks.resize(_canvas_chunk_total);
  }

  // Apply the chunk_data to the correct slot in the re-assembly buffer
  uint32_t idx = canvas_chunk_id - 1;
  if (idx >= _canvas_chunks.size()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas chunk id exceeds buffer");
    resetCanvasReassembly();
    return false;
  }

  // First time we've seen this chunk id? Copy the bytes into the buffer for reassembly.
  if (_canvas_chunks[idx].empty())
    _canvas_chunks_received++;
  // Also let's avoid copying bytes 2x
  _canvas_chunks[idx] = std::move(_pending_chunk);
  _pending_chunk.clear();

  return true;
}

/*!
    @brief  Concatenates every received chunk into the bitmap buffer (_bmp),
            draws it to the named display, and clears the reassembly state
            afterward.
    @param  msg  The Display Write message (provides name + descriptor).
    @return True if the display drew the bitmap, False otherwise.
*/
bool DisplayController::drawCanvasToDisplay(ws_display_Write *msg) {
  // Concatenate every region in chunk_id order into _bmp and validate its
  // total size. clear() first so a prior image's bytes never linger between
  // reassemblies.
  WS_DEBUG_PRINT("[display] Building BMP from chunks received: ");
  WS_DEBUG_PRINTVAR(_canvas_chunks_received);
  WS_DEBUG_PRINT(" / ");
  WS_DEBUG_PRINTLNVAR(_canvas_chunk_total);
  _bmp.clear();
  _bmp.reserve(std::min<size_t>(_canvas_total_size, MAX_CANVAS_SIZE));
  for (uint32_t i = 0; i < _canvas_chunk_total; i++) {
    _bmp.insert(_bmp.end(), _canvas_chunks[i].begin(), _canvas_chunks[i].end());
  }
  if (_bmp.size() != _canvas_total_size) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas size mismatch, discarding");
    resetCanvasReassembly();
    return false;
  }

  // Hand the assembled BMP to the target display's driver.
  WS_DEBUG_PRINT("[display] Writing assembled BMP to display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);
  int8_t disp_idx =
      resolveDisplayOrPublishError(msg, "Display not found for image write");
  if (disp_idx < 0) {
    resetCanvasReassembly();
    return false;
  }

  // Attempt to draw the assembled BMP to the display
  WS_DEBUG_PRINTLN("[display] Attempting to draw canvas to display...");
  bool did_draw = _displays[disp_idx]->drawMarqueeEPD(_bmp.data(), _bmp.size());
  WS_DEBUG_PRINT("[display] Draw result: ");
  WS_DEBUG_PRINTLNVAR(did_draw);
  resetCanvasReassembly(); // frees the per-chunk regions now that bmp is built
  if (!did_draw && msg->has_descriptor) {
    PublishDisplayComponentError(msg->descriptor,
                                 "Failed to draw canvas to display");
  }
  return did_draw;
}

/*!
    @brief  Resolves the target display for a write request by name.
    @param  msg  The Display Write message (provides name + descriptor).
    @param  errorMessage  Error published when the display is not found.
    @return The display index, or -1 if no matching display exists (in which
            case a component error is published when a descriptor is present).
*/
int8_t
DisplayController::resolveDisplayOrPublishError(ws_display_Write *msg,
                                                const char *errorMessage) {
  if (!msg->has_descriptor) {
    WS_DEBUG_PRINTLN("[display] WARNING: No descriptor in write message to "
                     "publish error against");
    return -1;
  }

  int8_t idx = findDisplayIndexByName(msg->name);
  if (idx < 0)
    PublishDisplayComponentError(msg->descriptor, errorMessage);
  return idx;
}

/*!
    @brief  Updates the status bar on all displays every 60 seconds.
    @param  rssi  The current WiFi RSSI.
    @param  is_connected  Whether MQTT is currently connected.
*/
void DisplayController::update(int32_t rssi, bool is_connected) {
  if (_num_displays == 0)
    return;

  unsigned long now = millis();
  if (now - _last_bar_update < ONE_MINUTE_IN_MS)
    return;
  _last_bar_update = now;
  // TODO: Get actual battery level if available
  uint8_t battery_charge_level = 100;
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i]) {
      WS_DEBUG_PRINTLN("[display] Updating status bar...");
      // TODO: TRICOLOR and QUADCOLOR (not sure about grayscale) displays take a
      // LONG time to refresh
      // TODO: maybe kill this functionality if they are actively using Marquee,
      //       and refresh the status bar along with the marquee?
      // _displays[i]->updateStatusBar(rssi, battery_charge_level,
      // is_connected);
    }
  }
}

/*!
    @brief  Finds a display by name.
    @param  name  The display name to search for.
    @return Index of the display, or -1 if not found.
*/
int8_t DisplayController::findDisplayIndexByName(const char *name) {
  if (!name) {
    WS_DEBUG_PRINTLN("[display] ERROR: Null display name provided for search");
    return -1;
  }
  for (uint8_t i = 0; i < _num_displays; i++) {
    // TODO: Remove, debug only print out the display names
    if (_displays[i]) {
      WS_DEBUG_PRINT("[display] Checking display at index ");
      WS_DEBUG_PRINTVAR(i);
      WS_DEBUG_PRINT(" with name: ");
      WS_DEBUG_PRINTLNVAR(_displays[i]->getName());
    }
    if (_displays[i] && strcmp(_displays[i]->getName(), name) == 0) {
      return i;
    }
  }
  return -1;
}
