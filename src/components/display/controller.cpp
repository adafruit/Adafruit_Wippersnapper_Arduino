/*!
 * @file src/components/display/controller.cpp
 *
 * Implementation for the display API controller.
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
#include "controller.h"

/*!
    @brief  Constructs a new DisplayController object
*/
DisplayController::DisplayController() {}

/*!
    @brief  Destructor
*/
DisplayController::~DisplayController() {
  // Clean up all display hardware instances
  for (DisplayHardware *hw_instance : _hw_instances) {
    delete hw_instance;
  }
  _hw_instances.clear();
}

/*!
    @brief  Handles a Display_AddOrReplace message.
    @param  msgAdd
            Pointer to a DisplayAddOrReplace message structure.
    @return True if the display was added or replaced successfully, false
   otherwise.
*/
bool DisplayController::Handle_Display_AddOrReplace(
    wippersnapper_display_v1_DisplayAddOrReplace *msgAdd) {
  DisplayHardware *display = new DisplayHardware(msgAdd->name);
  WS_DEBUG_PRINT("[display] Adding or replacing display: ");
  WS_DEBUG_PRINTLN(msgAdd->name);

  // Configure display type
  display->setType(msgAdd->type);

  // Attempt to initialize display hardware instance
  bool did_begin = false;
  if (msgAdd->which_config ==
      wippersnapper_display_v1_DisplayAddOrReplace_config_epd_tag) {
    did_begin = display->beginEPD(&msgAdd->config.config_epd,
                                  &msgAdd->interface_type.spi_epd);
  } else {
    WS_DEBUG_PRINTLN("[display] Unsupported display configuration type!");
    return false;
  }

  // Check if the display began successfully
  if (!did_begin) {
    WS_DEBUG_PRINTLN("[display] Failed to initialize display!");
    delete display; // Clean up if initialization failed
    return false;
  }

  _hw_instances.push_back(display); // Store the display instance
  WS_DEBUG_PRINTLN("[display] Display added or replaced successfully!");
  return true;
}

/*!
    @brief  Handles a Display_Remove message.
    @param  msgRemove
            Pointer to a DisplayRemove message structure.
    @return True if the display was removed successfully, false otherwise.
*/
bool DisplayController::Handle_Display_Remove(
    wippersnapper_display_v1_DisplayRemove *msgRemove) {
  // Find the display instance by name
  for (auto it = _hw_instances.begin(); it != _hw_instances.end(); ++it) {
    if (strcmp((*it)->getName(), msgRemove->name) == 0) {
      delete *it;
      _hw_instances.erase(it);
      WS_DEBUG_PRINTLN("[display] Display removed successfully!");
      return true;
    }
  }
  WS_DEBUG_PRINTLN("[display] Could not remove display, not found!");
  return false;
}

/*!
    @brief  Handles a Display_Write message.
    @param  msgWrite
            Pointer to a DisplayWrite message structure.
    @return True if the display write was successful, false otherwise.
*/
bool DisplayController::Handle_Display_Write(
    wippersnapper_display_v1_DisplayWrite *msgWrite) {
  // Get the driver instance for the display
  DisplayHardware *display = nullptr;
  for (auto &hw_instance : _hw_instances) {
    if (strcmp(hw_instance->getName(), msgWrite->name) == 0) {
      display = hw_instance;
      break;
    }
  }

  // Early-out if driver instance not found
  if (!display) {
    WS_DEBUG_PRINTLN("[display] Failed to write, driver not found!");
    return false;
  }

  // Write the message to the display
  WS_DEBUG_PRINT("[display] Writing message to display: ");
  WS_DEBUG_PRINTLN(msgWrite->message);
  display->writeMessage(msgWrite->message);
  return true;
}