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
DisplayController::DisplayController() {
  // TODO
}

/*!
    @brief  Destructor
*/
DisplayController::DisplayController() {
  // TODO
}

/*!
    @brief  Handles a Display_AddOrReplace message.
    @param  msgAdd
            Pointer to a DisplayAddOrReplace message structure.
    @return True if the display was added or replaced successfully, false otherwise.
*/
bool DisplayController::Handle_Display_AddOrReplace(wippersnapper_display_v1_DisplayAddOrReplace *msgAdd) {
    DisplayHardware *display = new DisplayHardware(msgAdd->name);

    // Configure display type
    display->setType(msgAdd->type);

    // Attempt to initialize display hardware instance
    bool did_begin = false;
    if (msgAdd->which_config == wippersnapper_display_v1_DisplayAddOrReplace_epd_config_tag) {
        did_begin = display->beginEPD(&msgAdd->config.epd_config, &msgAdd->interface_type.spi_epd);
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
    return true; // Placeholder
}
