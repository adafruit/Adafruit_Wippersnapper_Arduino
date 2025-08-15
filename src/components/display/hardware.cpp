/*!
 * @file src/components/display/hardware.cpp
 *
 * Implementation for the display hardware.
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
    @brief  Constructs a new DisplayHardware object
*/
DisplayHardware::DisplayHardware() {
  _type = wippersnapper_display_v1_DisplayType_DISPLAY_TYPE_UNSPECIFIED;
}

/*!
    @brief  Destructor
*/
DisplayHardware::DisplayHardware() {
  // TODO: Clean up display drivers
}

/*!
    @brief  Sets the hardware's display type.
    @param  type
            The display type to set.
*/
void DisplayHardware::setType(wippersnapper_display_v1_DisplayType type) {
    _type = type;
}

/*!
    @brief  Initializes the display hardware.
    @param  reset
            Whether to reset the display hardware.
    @return True if initialization was successful, false otherwise.
*/
bool DisplayHardware::begin(bool reset) {
    return true; // Placeholder for actual initialization logic
}

/*!
    @brief  Sets the text size for the display.
    @param  sz
            The size of the text to set.
*/
void setTextSize(uint8_t sz) {
    // Placeholder for setting text size on the display TODO
}

