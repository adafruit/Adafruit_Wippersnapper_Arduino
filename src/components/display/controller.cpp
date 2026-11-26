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
  case ws_display_B2D_modify_tag: {
    // Modify = Remove the old component, then Add the new one. Bail before
    // removing anything if there is no Add to replace it with.
    auto &m = b2d.payload.modify;
    if (!m.has_add) {
      Ws->error_handler->publishComponentError(
          m.has_remove ? m.remove.name : "",
          "Modify requires an Add, nothing was removed");
      return false;
    }
    if (m.has_remove) {
      (void)Handle_Display_Remove(&m.remove);
    }
    return Handle_Display_Add(&m.add);
  }
  case ws_display_B2D_write_tag:
    return Handle_Display_Write(&b2d.payload.write);
  default:
    WS_DEBUG_PRINTLN("[display] WARNING: Unsupported Display payload");
    return false;
  }
}

/*!
    @brief  Checks whether two interface descriptors identify the same
            physical display: same interface type and the same select line
            (SPI bus + CS, I2C bus pins + address, i8080 CS, DSI bus). An
            RGB666 dot-clock panel is unique per board.
    @param  a  First interface descriptor.
    @param  b  Second interface descriptor.
    @return True if both describe the same display, False otherwise.
*/
static bool sameDisplayDescriptor(const ws_display_InterfaceDescriptor *a,
                                  const ws_display_InterfaceDescriptor *b) {
  if (!a || !b || a->which_descriptor != b->which_descriptor)
    return false;
  switch (a->which_descriptor) {
  case ws_display_InterfaceDescriptor_spi_tag:
    return a->descriptor.spi.spi.bus == b->descriptor.spi.spi.bus &&
           strcmp(a->descriptor.spi.spi.pin_cs, b->descriptor.spi.spi.pin_cs) ==
               0;
  case ws_display_InterfaceDescriptor_i2c_tag:
    return a->descriptor.i2c.address == b->descriptor.i2c.address &&
           strcmp(a->descriptor.i2c.address_space.pin_scl,
                  b->descriptor.i2c.address_space.pin_scl) == 0 &&
           strcmp(a->descriptor.i2c.address_space.pin_sda,
                  b->descriptor.i2c.address_space.pin_sda) == 0 &&
           a->descriptor.i2c.address_space.mux_address ==
               b->descriptor.i2c.address_space.mux_address &&
           a->descriptor.i2c.address_space.mux_channel ==
               b->descriptor.i2c.address_space.mux_channel;
  case ws_display_InterfaceDescriptor_ttl_rgb666_tag:
    return true;
  case ws_display_InterfaceDescriptor_i8080_tag:
    return strcmp(a->descriptor.i8080.pin_cs, b->descriptor.i8080.pin_cs) == 0;
  case ws_display_InterfaceDescriptor_dsi_tag:
    return a->descriptor.dsi.bus == b->descriptor.dsi.bus;
  default:
    return false;
  }
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

  // Driver, panel and mode come from the component definition (Add.driver,
  // Add.panel, config), not from the component name.

  // If a display on the same interface exists, remove it first to allow
  // replacement. Identical components on other pins/addresses stay.
  if (removeExistingDisplay(msg->name, msg->has_interface_type
                                           ? &msg->interface_type
                                           : nullptr)) {
    WS_DEBUG_PRINT("[display] Replaced existing display on the same "
                   "interface as '");
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
  hw->initialise(Ws->_configV2.aio_user);

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
  case ws_display_InterfaceDescriptor_spi_tag:
    Ws->error_handler->publishComponentError(iface.descriptor.spi.spi, error);
    break;
  case ws_display_InterfaceDescriptor_i2c_tag:
    Ws->error_handler->publishComponentError(iface.descriptor.i2c, error);
    break;
  case ws_display_InterfaceDescriptor_ttl_rgb666_tag:
    Ws->error_handler->publishComponentError(iface.descriptor.ttl_rgb666.pin_b0,
                                             error);
    break;
  case ws_display_InterfaceDescriptor_i8080_tag:
    Ws->error_handler->publishComponentError(iface.descriptor.i8080.pin_d0,
                                             error);
    break;
  case ws_display_InterfaceDescriptor_dsi_tag:
    Ws->error_handler->publishComponentError(iface.descriptor.dsi.pin_rst,
                                             error);
    break;
  default:
    WS_DEBUG_PRINTLN(
        "[display] WARNING: Unknown interface type in add request");
    Ws->error_handler->publishComponentError("Unknown interface", error);
    break;
  }
}

/*!
    @brief  Removes an existing display, identified by its interface
            descriptor (or by component name when no descriptor is given).
            Adding a display on an occupied interface replaces the old one
            instead of creating a conflict.
    @param  name  The component name, used only when desc is null.
    @param  desc  The interface descriptor of the display, or nullptr.
    @return True if a display was removed, False if none matched.
*/
bool DisplayController::removeExistingDisplay(
    const char *name, const ws_display_InterfaceDescriptor *desc) {
  int8_t existingIdx =
      desc ? findDisplayIndexByDescriptor(desc) : findDisplayIndexByName(name);
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
  bool did_remove = removeExistingDisplay(
      msg->name, msg->has_descriptor ? &msg->descriptor : nullptr);
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

  int8_t idx = msg->has_descriptor
                   ? findDisplayIndexByDescriptor(&msg->descriptor)
                   : findDisplayIndexByName(msg->name);
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
    @brief  Finds a display by its interface descriptor (bus + select pin or
            address), so identical components on different pins are
            distinct displays.
    @param  desc  The interface descriptor to search for.
    @return Index of the display, or -1 if not found.
*/
int8_t DisplayController::findDisplayIndexByDescriptor(
    const ws_display_InterfaceDescriptor *desc) {
  if (!desc)
    return -1;
  for (uint8_t i = 0; i < _num_displays; i++) {
    if (_displays[i] &&
        sameDisplayDescriptor(_displays[i]->getInterface(), desc)) {
      return i;
    }
  }
  return -1;
}

/*!
    @brief  Finds a display by component name. Fallback for messages that
            carry no interface descriptor.
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
