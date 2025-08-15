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

bool DisplayController::Handle_Display_AddOrReplace(wippersnapper_display_v1_DisplayAddOrReplace *msgAdd) {
    DisplayHardware *display = new DisplayHardware();
    // Configure display type
    display->setType(msgAdd->type);
    // Configure display based on config type
    if (msgAdd->which_config == wippersnapper_display_v1_DisplayAddOrReplace_epd_config_tag) {
        display->configureEPD(&msgAdd->config.epd_config, &msgAdd->interface_type.spi_epd);
    }
    return true; // Placeholder
}
