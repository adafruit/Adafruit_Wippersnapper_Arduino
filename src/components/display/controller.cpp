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
DisplayController::DisplayController() { _last_bar_update = 0; }

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
  WS_DEBUG_PRINTLNVAR(msgAdd->name);

  // Does this display hw instance already exist?
  DisplayHardware *existingDisplay = findDisplay(msgAdd->name);
  if (existingDisplay != nullptr) {
    WS_DEBUG_PRINTLN("[display] Display exists, removing...");
    for (std::vector<DisplayHardware *>::iterator it = _hw_instances.begin();
         it != _hw_instances.end(); ++it) {
      if (*it == existingDisplay) {
        delete *it;
        _hw_instances.erase(it);
        break;
      }
    }
  }

  // Configure display type
  display->setType(msgAdd->type);

  // Attempt to initialize display hardware instance
  bool did_begin = false;
  if (msgAdd->which_config ==
      wippersnapper_display_v1_DisplayAddOrReplace_config_epd_tag) {
    did_begin = display->beginEPD(&msgAdd->driver, &msgAdd->config.config_epd,
                                  &msgAdd->interface_type.spi_epd);
  } else if (msgAdd->which_config ==
             wippersnapper_display_v1_DisplayAddOrReplace_config_tft_tag) {
    did_begin = display->beginTft(&msgAdd->driver, &msgAdd->config.config_tft,
                                  &msgAdd->interface_type.spi_tft);
  } else {
    WS_DEBUG_PRINTLN("[display] Unsupported display configuration type!");
    delete display;
    return false;
  }

  // Check if the display began successfully
  if (!did_begin) {
    WS_DEBUG_PRINTLN("[display] Failed to initialize display!");
    delete display; // Clean up if initialization failed
    return false;
  }

  WS.runNetFSM();
  display->showSplash();
  WS.runNetFSM();
  display->drawStatusBar(WS._config.aio_user);
  WS.runNetFSM();

  _hw_instances.push_back(display); // Store the display instance
  WS_DEBUG_PRINTLN("[display] Display added or replaced successfully!");
  WS.runNetFSM();
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
  if (!msgRemove || !msgRemove->name)
    return false;

  DisplayHardware *display = findDisplay(msgRemove->name);

  if (display == nullptr)
    return false; // Display not found

  // Remove from vector
  for (std::vector<DisplayHardware *>::iterator it = _hw_instances.begin();
       it != _hw_instances.end(); ++it) {
    if (*it == display) {
      delete *it;
      _hw_instances.erase(it);
      WS_DEBUG_PRINTLN("[display] Display removed successfully!");
      return true;
    }
  }

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
  DisplayHardware *display = findDisplay(msgWrite->name);

  // Early-out if driver instance not found
  if (!display) {
    WS_DEBUG_PRINTLN("[display] Failed to write, driver not found!");
    return false;
  }

  // Write the message to the display
  WS_DEBUG_PRINT("[display] Writing message to display: ");
  WS_DEBUG_PRINTLNVAR(msgWrite->message);
  WS.runNetFSM();
  display->writeMessage(msgWrite->message);
  WS.runNetFSM();
  return true;
}

/*!
    @brief  Updates the status bar on all managed displays.
    @param  rssi
            The current WiFi RSSI value.
    @param  is_connected
            The current MQTT connection status.
*/
void DisplayController::update(int32_t rssi, bool is_connected) {
  // if _hw_instances is empty, early out
  if (_hw_instances.size() == 0)
    return;

  // Only update the status bar every 60 seconds
  unsigned long now = millis();
  if (now - _last_bar_update < 60000)
    return;
  _last_bar_update = now;

  // Get the driver instance for the display
  for (DisplayHardware *hw_instance : _hw_instances) {
    // Note: For now, battery is always 100% as we don't have a way to read it
    // yet.
    WS_DEBUG_PRINTLN("[display] Updating status bar...");
    hw_instance->updateStatusBar(rssi, 100, is_connected);
    WS.runNetFSM();
  }
}

/*!
 * @brief Finds a DisplayHardware instance by its name.
 * @param name The name of the display to find.
 * @return Pointer to the DisplayHardware instance if found, nullptr otherwise.
 */
DisplayHardware *DisplayController::findDisplay(const char *name) {
  if (name == nullptr)
    return nullptr;

  for (std::vector<DisplayHardware *>::iterator it = _hw_instances.begin();
       it != _hw_instances.end(); ++it) {
    if (*it != nullptr && strcmp((*it)->getName(), name) == 0) {
      return *it;
    }
  }

  return nullptr;
}