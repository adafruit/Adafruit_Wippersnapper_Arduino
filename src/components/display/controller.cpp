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
  /*!
    @brief  Initializes the ThinkInk panel factory table to avoid a stack
    overflow.
  */
  drvDispThinkInk::getAdafruitEPDFactory();
  _num_displays = 0;
  _last_bar_update = 0;
  _canvas_checksum = 0;
  _image_chunk_total = 0;
  _image_chunks_received = 0;
  _image_chunk_bytes = 0;
  _expected_image_size = 0;
  _marquee_state = marquee_state_t::IDLE;
  _prv_image_activity = 0;
}

DisplayController::~DisplayController() {
  for (int i = 0; i < _num_displays; i++) {
    delete _displays[i];
  }
  _num_displays = 0;
  resetImage();
}

/*!
    @brief  Frees all buffered canvas chunk regions and the assembled bitmap,
            and zeroes the per-canvas reassembly state. Does NOT touch
            _pending_chunk: that holds the current message's just-decoded bytes
            and is owned by the decode callback / Handle_Display_Write, which
            may call this mid-message (e.g. on a new image id) before filing
            those bytes.
*/
void DisplayController::resetImage() {
  // swap-with-empty releases the heap capacity, not just the size
  std::vector<std::vector<uint8_t>>().swap(_image_chunks);
  std::vector<uint8_t>().swap(_bmp);
  _canvas_checksum = 0;
  _image_chunk_total = 0;
  _image_chunks_received = 0;
  _image_chunk_bytes = 0;
  _expected_image_size = 0;
}

/*!
    @brief  Routes messages using the display.proto API to the
            appropriate controller functions.
    @param  stream  The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool DisplayController::Router(pb_istream_t *stream) {
  // Save the stream before decoding — the write case re-decodes with a
  // chunk_data callback wired up, and ws_pb_decode consumes the stream.
  pb_istream_t saved_stream = *stream;

  // ws_display_B2D is large (~1.4KB — its union embeds Write.message[1024]).
  // Router runs nested inside the outer signal pb_decode (via the cb_payload
  // callback), so keeping this struct on the stack risks overflowing the
  // loopTask stack. Heap-allocate it and free it before every return.
  ws_display_B2D *b2d = new ws_display_B2D;
  memset(b2d, 0, sizeof(ws_display_B2D));
  if (!ws_pb_decode(stream, ws_display_B2D_fields, b2d)) {
    WS_DEBUG_PRINTLN("[display] ERROR: Unable to decode Display B2D envelope");
    delete b2d;
    return false;
  }

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
    // Re-decode from the saved stream with the image write callback present
    // b2d is not required for the re-decode, the following avoids extra heap
    // usage
    delete b2d;
    b2d = nullptr;
    // Allocate a new b2d for the re-decode
    ws_display_B2D *b2d_w = new ws_display_B2D;
    memset(b2d_w, 0, sizeof(ws_display_B2D));
    b2d_w->which_payload = ws_display_B2D_write_tag;
    b2d_w->payload.write.image.chunk_data.funcs.decode = cbDecodeImageChunk;
    b2d_w->payload.write.image.chunk_data.arg = this;
    if (!pb_decode_ex(&saved_stream, ws_display_B2D_fields, b2d_w,
                      PB_DECODE_NOINIT)) {
      WS_DEBUG_PRINT("[display] ERROR: Failed to re-decode write w/ canvas: ");
      WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(&saved_stream));
      delete b2d_w;
      // delete b2d;
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
  if (!hw->isMarquee()) {
    hw->initialise(Ws._configV2.aio_user);
  }

  _displays[_num_displays] = hw;
  _num_displays++;

  // The broker pushes an EPD's content as a separate write every wake cycle,
  // arriving after check-in completes.
  if (msg->type == ws_display_DisplayClass_DISPLAY_CLASS_EPD) {
    _marquee_state = marquee_state_t::AWAITING;
    // Enable the best-effort idle timeout for the 1st chunk of a new image
    _prv_image_activity = millis();
  }

  // Handle optional initial write
  if (msg->has_write) {
    WS_DEBUG_PRINTLN("[display] Processing initial write...");
    // Only TEXT writes are supported on this path today, not images
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
  _marquee_state = marquee_state_t::IDLE;
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
bool DisplayController::cbDecodeImageChunk(pb_istream_t *stream,
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
    return handleImageWrite(msg);
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

  int8_t idx =
      resolveDisplayOrPublishError(msg, "Display not found for write request");
  if (idx < 0) {
    return false;
  }

  bool did_write = _displays[idx]->write(msg);
  if (did_write) {
    // Content for this wake has been drawn; release the sleep hold-off.
    _marquee_state = marquee_state_t::IDLE;
  }
  return did_write;
}

/*!
    @brief  Ingests one canvas chunk and, once every chunk has arrived,
            reassembles the full BMP and draws it to the target display.
    @param  msg  The Display Write message carrying the image chunk.
    @return True if the chunk was accepted (or the canvas drawn), False on
            error.
*/
bool DisplayController::handleImageWrite(ws_display_Write *msg) {
  // Ingest this chunk of data, if we're not done ingesting the full canvas yet.
  if (!processImageChunk(msg)) {
    return false;
  }

  // Wait for more chunks before assembling.
  if (_image_chunks_received < _image_chunk_total)
    return true;

  // All chunks received: assemble the full BMP and draw it to the display.
  _marquee_state = marquee_state_t::DRAWING;
  bool did_draw = drawImage(msg);
  if (!did_draw) {
    // Exhausted all attempts to draw the image to the display.
    if (msg->has_descriptor) {
      PublishDisplayComponentError(msg->descriptor,
                                   "Failed to draw canvas to display");
    }
    // Reset the reassembly state so the device can sleep
    _marquee_state = marquee_state_t::IDLE;
    return false;
  }

  // Publish the WriteComplete D2B message to tell the broker that the panel
  // fully received the canvas.
  _prv_image_activity = millis();
  if (!publishWriteComplete()) {
    WS_DEBUG_PRINTLN("[display] Inline WriteComplete publish FAILED");
  }
  _marquee_state = marquee_state_t::IDLE;

  return true;
}

/*!
    @brief  Publishes the display WriteComplete D2B message, telling the broker
            that the panel fully received the canvas.
    @return True if the message was published, False otherwise.
*/
bool DisplayController::publishWriteComplete() {
  if (Ws._sdCardV2->isModeOffline() || Ws._mqttV2 == nullptr)
    return false;

  WS_DEBUG_PRINT("[display] Publishing WriteComplete D2B... MQTT "
                 "connected: ");
  WS_DEBUG_PRINTLNVAR(Ws._mqttV2->connected());

  ws_display_D2B write_complete_d2b = ws_display_D2B_init_zero;
  write_complete_d2b.which_payload = ws_display_D2B_write_complete_tag;
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_display_tag,
                     &write_complete_d2b)) {
    WS_DEBUG_PRINTLN("[display] WriteComplete publish FAILED");
    return false;
  }

  WS_DEBUG_PRINTLN("[display] WriteComplete PUBLISHED!");
  return true;
}

/*!
    @brief  Validates and files a single canvas chunk into the reassembly
            buffer, tracking how many distinct chunks have arrived.
    @param  msg  The Display Write message carrying the image chunk.
    @return False on validation failure (reassembly state is reset before
            returning), True if the chunk was accepted.
*/
bool DisplayController::processImageChunk(ws_display_Write *msg) {
  // Did the decode callback actually decode any bytes into the chunk?
  if (_pending_chunk.empty()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas chunk_data missing/empty");
    resetImage();
    return false;
  }

  // Split out the chunk metadata
  uint32_t canvas_checksum = msg->image.checksum;
  uint32_t sz_image = msg->image.size;
  uint32_t canvas_chunk_id = msg->image.chunk_id;
  uint32_t canvas_chunk_total = msg->image.chunk_total;

  // Validate the chunk metadata
  if (canvas_chunk_total == 0 || canvas_chunk_total > MAX_CANVAS_CHUNKS ||
      canvas_chunk_id < 1 || canvas_chunk_id > canvas_chunk_total) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas chunk id/total out of range");
    resetImage();
    return false;
  }

  // If the image is too large, gracefully bail out
  if (sz_image == 0 || sz_image > (uint32_t)MAX_CANVAS_SIZE) {
    WS_DEBUG_PRINT("[display] ERROR: Expected image size out of range: ");
    WS_DEBUG_PRINTLNVAR(sz_image);
    resetImage();
    return false;
  }

  // New incoming image? Reset the reassembly state and start over.
  if (_image_chunks.empty() || canvas_checksum != _canvas_checksum) {
    resetImage();
    _canvas_checksum = canvas_checksum;
    _image_chunk_total = canvas_chunk_total;
    _expected_image_size = sz_image;
    _image_chunks.resize(_image_chunk_total);
  }

  // Apply the chunk_data to the correct slot in the re-assembly buffer
  uint32_t idx = canvas_chunk_id - 1;
  if (idx >= _image_chunks.size()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas chunk id exceeds buffer");
    resetImage();
    return false;
  }

  // First time we've seen this chunk id? Copy the bytes into the buffer for
  // reassembly.
  if (_image_chunks[idx].empty())
    _image_chunks_received++;
  _image_chunk_bytes -= _image_chunks[idx].size();
  // Also let's avoid copying bytes 2x
  _image_chunks[idx] = std::move(_pending_chunk);
  _image_chunk_bytes += _image_chunks[idx].size();
  _pending_chunk.clear();

  // Re-arm the long per-packet MQTT idle timeout: chunks are actively arriving,
  // so the next read should get the wide allowance rather than the short poll.
  _prv_image_activity = millis();

  _marquee_state = marquee_state_t::STREAMING;
  return true;
}

/*!
    @brief  Concatenates every received chunk into the bitmap buffer (_bmp),
            draws it to the named .
    @param  msg  The Display Write message (provides name + descriptor).
    @return True if the display drew the bitmap, False if draw attempts failed.
*/
bool DisplayController::drawImage(ws_display_Write *msg) {
  // Concatenate every region in chunk_id order into _bmp and validate its
  // total size. clear() first so a prior image's bytes never linger between
  // reassemblies.
  WS_DEBUG_PRINT("[display] Building BMP from chunks received: ");
  WS_DEBUG_PRINTVAR(_image_chunks_received);
  WS_DEBUG_PRINT(" / ");
  WS_DEBUG_PRINTLNVAR(_image_chunk_total);
  // Reserve what actually arrived, not the declared size: _image_chunk_bytes
  // only counts bytes already held in _image_chunks, so this cannot ask for
  // more than the heap has already given us, and it allocates once instead of
  // letting insert() grow geometrically. A declared size that disagrees is
  // caught by the mismatch check below.
  _bmp.clear();
  _bmp.reserve(_image_chunk_bytes);
  for (uint32_t i = 0; i < _image_chunk_total; i++) {
    _bmp.insert(_bmp.end(), _image_chunks[i].begin(), _image_chunks[i].end());
    // Release each chunk as it is consumed so the assembled bitmap and the
    // chunk buffers do not both hold a full copy of the canvas at once. Every
    // exit path below calls resetImage(), so these regions are not needed
    // again. swap-with-empty releases the capacity, not just the size.
    std::vector<uint8_t>().swap(_image_chunks[i]);
  }
  if (_bmp.size() != _expected_image_size) {
    WS_DEBUG_PRINTLN("[display] ERROR: Canvas size mismatch, discarding");
    resetImage();
    return false;
  }

  // Hand the assembled BMP to the target display's driver.
  WS_DEBUG_PRINT("[display] Writing assembled BMP to display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);
  int8_t disp_idx =
      resolveDisplayOrPublishError(msg, "Display not found for image write");
  if (disp_idx < 0) {
    resetImage();
    return false;
  }

  // Attempt to draw the assembled BMP to the display
  bool did_draw = false;
  for (uint8_t img_draw_attempts = 1;
       img_draw_attempts <= MAX_CANVAS_DRAW_ATTEMPTS; img_draw_attempts++) {
    did_draw = _displays[disp_idx]->drawMarqueeEPD(_bmp.data(), _bmp.size());
    if (did_draw)
      break;
  }
  resetImage();

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

  // If the display is a "marquee/canvas" type, the broker is responsible for
  // drawing the status bar, so skip this.
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i]->isMarquee())
      return;
  }

  unsigned long now = millis();
  if (now - _last_bar_update < ONE_MINUTE_IN_MS)
    return;
  _last_bar_update = now;
  // TODO: Get actual battery level if available
  uint8_t battery_charge_level = 100;
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i]) {
      _displays[i]->updateStatusBar(rssi, battery_charge_level, is_connected);
    }
  }
}

/*!
    @brief  Finds a display by name.
    @param  name  The display name to search for.
    @return Index of the display, or -1 if not found.
*/
int8_t DisplayController::findDisplayIndexByName(const char *name) {
  if (!name)
    return -1;

  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i] && strcmp(_displays[i]->getName(), name) == 0)
      return i;
  }
  return -1;
}

/*!
    @brief  Reports whether the display controller has any marquee work
            outstanding, so loopSleep() can treat it like every other polled
            controller. Every state other than IDLE means a canvas is expected,
            in flight, or mid-draw.
    @return True if there is nothing left to do this wake, False otherwise.
*/
bool DisplayController::UpdateComplete() {
  return _marquee_state == marquee_state_t::IDLE;
}

/*!
    @brief  Clears the image's state for the next loopSleep() cycle.
*/
void DisplayController::ResetFlags() {
  resetImage();
  _marquee_state = marquee_state_t::IDLE;
}

/*!
    @brief  Whether image write packets are currently in-flight.
    @return True if an image is currently being streamed to the display, False
   otherwise.
*/
bool DisplayController::isImageStreaming() {
  if (_marquee_state == marquee_state_t::IDLE)
    return false;
  return (millis() - _prv_image_activity) < CANVAS_STREAM_IDLE_MS;
}
