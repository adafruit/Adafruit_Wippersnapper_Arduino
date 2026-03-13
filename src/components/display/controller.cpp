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
}

DisplayController::~DisplayController() {
  for (int i = 0; i < _num_displays; i++) {
    delete _displays[i];
  }
  _num_displays = 0;
}

/*!
    @brief  Routes messages using the display.proto API to the
            appropriate controller functions.
    @param  stream  The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool DisplayController::Router(pb_istream_t *stream) {
  ws_display_B2D b2d = ws_display_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_display_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[display] ERROR: Unable to decode Display B2D envelope");
    return false;
  }

  switch (b2d.which_payload) {
  case ws_display_B2D_add_tag:
    return Handle_Display_Add(&b2d.payload.add);
  case ws_display_B2D_remove_tag:
    return Handle_Display_Remove(&b2d.payload.remove);
  case ws_display_B2D_write_tag:
    return Handle_Display_Write(&b2d.payload.write);
  default:
    WS_DEBUG_PRINTLN("[display] WARNING: Unsupported Display payload");
    return false;
  }
}

/*!
    @brief  Handles a request to add or replace a display.
    @param  msg  The Display Add message.
    @return True if successful, False otherwise.
*/
/*!
    @brief  Resolves component-name based driver/mode defaults for EPD displays.
            Called before passing the Add message to hardware so that the
            controller owns the "what driver" decision and hardware just inits.
    @param  msg  The Display Add message (may be modified in place).
*/
static void resolveEpdDefaults(ws_display_Add *msg) {
  if (msg->which_interface_type != ws_display_Add_spi_epd_tag)
    return;
  if (msg->which_config != ws_display_Add_config_epd_tag)
    return;

  ws_display_EPDConfig *config = &msg->config.config_epd;
  const char *name = msg->name;

  // MagTag auto-detection is handled at hardware level (needs SPI probing),
  // but we can still set mode default
  if (strncmp(name, "eink-magtag", 11) == 0) {
    if (config->mode == ws_display_EPDMode_EPD_MODE_UNSPECIFIED)
      config->mode = ws_display_EPDMode_EPD_MODE_GRAYSCALE4;
    return;
  }

  // Map specific component names to driver + default mode
  struct {
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

  for (auto &m : mappings) {
    if (strncmp(name, m.component, strlen(m.component)) == 0) {
      strncpy(msg->driver, m.driver, sizeof(msg->driver) - 1);
      msg->driver[sizeof(msg->driver) - 1] = '\0';
      if (config->mode == ws_display_EPDMode_EPD_MODE_UNSPECIFIED)
        config->mode = m.mode;
      WS_DEBUG_PRINT("[display] Resolved component '");
      WS_DEBUG_PRINTVAR(name);
      WS_DEBUG_PRINT("' -> driver '");
      WS_DEBUG_PRINTVAR(msg->driver);
      WS_DEBUG_PRINTLN("'");
      return;
    }
  }
}

/*!
    @brief  Resolves component-name based driver/panel defaults for RGB666
            dotclock displays (Qualia).
    @param  msg  The Display Add message (may be modified in place).
*/
static void resolveRgb666Defaults(ws_display_Add *msg) {
  if (msg->which_interface_type != ws_display_Add_ttl_rgb666_tag)
    return;
  if (msg->which_config != ws_display_Add_config_display_tag)
    return;

  const char *name = msg->name;

  struct {
    const char *component;
    const char *driver;
    const char *panel;
  } mappings[] = {
      {"qualia-round-21-480x480", "ST7701S", "TL021WVC02"},
      {"qualia-bar-32-320x820", "ST7701S", "TL032FWV01"},
  };

  for (auto &m : mappings) {
    if (strncmp(name, m.component, strlen(m.component)) == 0) {
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
      return;
    }
  }
}

bool DisplayController::Handle_Display_Add(ws_display_Add *msg) {
  WS_DEBUG_PRINT("[display] Adding display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  // Resolve component-name defaults before passing to hardware
  if (msg->type == ws_display_DisplayClass_DISPLAY_CLASS_EPD) {
    resolveEpdDefaults(msg);
  } else if (msg->type == ws_display_DisplayClass_DISPLAY_CLASS_TFT &&
             msg->which_interface_type == ws_display_Add_ttl_rgb666_tag) {
    resolveRgb666Defaults(msg);
  }

  removeExistingDisplayByName(msg->name);

  if (_num_displays >= MAX_DISPLAYS) {
    WS_DEBUG_PRINTLN("[display] ERROR: Maximum number of displays reached!");
    return false;
  }

  // Create and initialize new display hardware
  DisplayHardware *hw = new DisplayHardware();
  if (!hw->begin(msg)) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to initialize display hardware!");
    delete hw;
    // Publish failure response
    ws_display_D2B d2b = ws_display_D2B_init_zero;
    d2b.which_payload = ws_display_D2B_added_or_replaced_tag;
    d2b.payload.added_or_replaced.did_add = false;
    strncpy(d2b.payload.added_or_replaced.name, msg->name,
            sizeof(d2b.payload.added_or_replaced.name) - 1);
    Ws.PublishD2b(ws_signal_DeviceToBroker_display_tag, &d2b);
    return false;
  }

  // Show splash screen and status bar
  hw->showSplash();
  hw->drawStatusBar(Ws._configV2.aio_user);

  _displays[_num_displays] = hw;
  _num_displays++;

  // Handle optional initial write
  if (msg->has_write) {
    WS_DEBUG_PRINTLN("[display] Processing initial write...");
    Handle_Display_Write(&msg->write);
  }

  // Publish success response
  ws_display_D2B d2b = ws_display_D2B_init_zero;
  d2b.which_payload = ws_display_D2B_added_or_replaced_tag;
  d2b.payload.added_or_replaced.did_add = true;
  strncpy(d2b.payload.added_or_replaced.name, msg->name,
          sizeof(d2b.payload.added_or_replaced.name) - 1);
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_display_tag, &d2b)) {
    WS_DEBUG_PRINTLN("[display] WARNING: Failed to publish AddedOrReplaced");
  }

  WS_DEBUG_PRINT("[display] Display added successfully: ");
  WS_DEBUG_PRINTLNVAR(msg->name);
  return true;
}

/*!
    @brief  Removes an existing display with the same name as the new one being
   added. This ensures that adding a display with a duplicate name will replace
            the old one instead of creating a conflict.
    @param  name  The name of the display to remove.
    @return True if a display was removed, False if no existing display had the
   same name.
*/
bool DisplayController::removeExistingDisplayByName(char *name) {
  // If display with same name exists, remove it first
  int8_t existingIdx = findDisplayByName(name);
  if (existingIdx >= 0) {
    WS_DEBUG_PRINTLN("[display] Replacing existing display");
    delete _displays[existingIdx];
    _displays[existingIdx] = nullptr;
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
    @param  msg  The Display Remove message.
    @return True if successful, False otherwise.
*/
bool DisplayController::Handle_Display_Remove(ws_display_Remove *msg) {
  WS_DEBUG_PRINT("[display] Removing display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  bool did_remove = removeExistingDisplayByName(msg->name);

  if (!did_remove) {
    WS_DEBUG_PRINTLN("[display] WARNING: Display not found");
  }

  // Publish response
  ws_display_D2B d2b = ws_display_D2B_init_zero;
  d2b.which_payload = ws_display_D2B_removed_tag;
  d2b.payload.removed.did_remove = did_remove;
  strncpy(d2b.payload.removed.name, msg->name,
          sizeof(d2b.payload.removed.name) - 1);
  Ws.PublishD2b(ws_signal_DeviceToBroker_display_tag, &d2b);
  return did_remove;
}

/*!
    @brief  Handles a request to write to a display.
    @param  msg  The Display Write message.
    @return True if successful, False otherwise.
*/
bool DisplayController::Handle_Display_Write(ws_display_Write *msg) {
  WS_DEBUG_PRINT("[display] Writing to display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  int8_t idx = findDisplayByName(msg->name);
  if (idx < 0) {
    WS_DEBUG_PRINTLN("[display] ERROR: Display not found!");
    return false;
  }

  return _displays[idx]->write(msg);
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
  if (now - _last_bar_update < 60000)
    return;
  _last_bar_update = now;

  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i]) {
      WS_DEBUG_PRINTLN("[display] Updating status bar...");
      _displays[i]->updateStatusBar(rssi, 100, is_connected);
    }
  }
}

/*!
    @brief  Finds a display by name.
    @param  name  The display name to search for.
    @return Index of the display, or -1 if not found.
*/
int8_t DisplayController::findDisplayByName(const char *name) {
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i] && strcmp(_displays[i]->getName(), name) == 0) {
      return i;
    }
  }
  return -1;
}
