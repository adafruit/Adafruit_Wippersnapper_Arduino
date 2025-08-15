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
    @brief  Gets the hardware's display type.
    @return The current display type.
*/
wippersnapper_display_v1_DisplayType DisplayHardware::getType() {
    return _type;
}

/*!
    @brief  Configures the EPD display with the provided configuration.
    @param  config
            Pointer to the EPD configuration structure.
    @param  spi_config
            Pointer to the SPI configuration structure for EPD.
    @return True if configuration was successful, False otherwise.
*/
bool DisplayHardware::beginEPD(wippersnapper_display_v1_EPDConfig *config, wippersnapper_display_v1_EpdSpiConfig *spi_config) {
    // Convert pins in config to int16_t instances
    int16_t dc = -1, rst = -1, cs = -1, srcs = -1, busy = -1;
    dc = (int16_t)atoi(spi_config->pin_dc+ 1);
    rst = (int16_t)atoi(spi_config->pin_rst+ 1);
    cs = (int16_t)atoi(spi_config->pin_cs+ 1);
    srcs = (int16_t)atoi(spi_config->pin_sram_cs+ 1);
    busy = (int16_t)atoi(spi_config->pin_busy+ 1);

    // Configure EPD mode
    thinkinkmode_t epd_mode;
    if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_GRAYSCALE4) {
        epd_mode = THINKINK_GRAYSCALE4;
    } else if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_MONO) {
        epd_mode = THINKINK_MONO;
    } else {
        epd_mode = THINKINK_MONO; // Default to mono
    }

    // Assign driver instance based on panel type
    if (config->panel == wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_213_GRAYSCALE4_MFGN) {
        _disp_thinkink_grayscale4_eaamfgn = new ThinkInk_290_Grayscale4_EAAMFGN(dc, rst, cs, srcs, busy);
        _disp_thinkink_grayscale4_eaamfgn->begin(epd_mode);
    } else if (config->panel == wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_213_GRAYSCALE4_T5) {
        _disp_thinkink_grayscale4_t5 = new ThinkInk_290_Grayscale4_T5(dc, rst, cs, srcs, busy);
        _disp_thinkink_grayscale4_t5->begin(epd_mode);
    } else {
        return false; // Unsupported panel type
    }

    return true; // Configuration successful
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

