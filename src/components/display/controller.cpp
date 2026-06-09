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
  // B2D envelope carries the display name as a callback field
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
    WS_DEBUG_PRINTLN(
        "[display] WARNING: Unknown interface type in add request");
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
    @brief  Handles a request to write to a display.
    @param  msg  The Display Write message.
    @return True if successful, False otherwise.
*/
bool DisplayController::Handle_Display_Write(ws_display_Write *msg) {
  if (!msg || !msg->name) {
    WS_DEBUG_PRINTLN("[display] ERROR: Invalid display write request!");
    return false;
  }

  WS_DEBUG_PRINT("[display] Writing to display: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  int8_t idx = findDisplayIndexByName(msg->name);
  if (idx < 0) {
    WS_DEBUG_PRINT("[display] ERROR: Display (");
    WS_DEBUG_PRINTVAR(msg->name);
    WS_DEBUG_PRINTLN(") not found!");
    if (msg->has_descriptor) {
      PublishDisplayComponentError(msg->descriptor,
                                   "Display not found for write request");
    } else {
      WS_DEBUG_PRINTLN("[display] WARNING: No descriptor in write message to "
                       "publish error against");
    }
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
  if (now - _last_bar_update < ONE_MINUTE_IN_MS)
    return;
  _last_bar_update = now;
  // TODO: Get actual battery level if available
  uint8_t battery_charge_level = 100;
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i]) {
      WS_DEBUG_PRINTLN("[display] Updating status bar...");
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
  if (!name) {
    WS_DEBUG_PRINTLN("[display] ERROR: Null display name provided for search");
    return -1;
  }
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i] && strcmp(_displays[i]->getName(), name) == 0) {
      return i;
    }
  }
  return -1;
}
