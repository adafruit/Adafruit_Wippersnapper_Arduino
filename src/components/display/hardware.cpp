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
DisplayHardware::DisplayHardware(const char *name) {
  _name = name; ///< Set the name of the hardware instance
  _type = wippersnapper_display_v1_DisplayType_DISPLAY_TYPE_UNSPECIFIED;
}

/*!
    @brief  Destructor
*/
DisplayHardware::~DisplayHardware() {
  // TODO
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
bool DisplayHardware::beginEPD(
    wippersnapper_display_v1_EPDConfig *config,
    wippersnapper_display_v1_EpdSpiConfig *spi_config) {
  // Validate pointers
  if (config == nullptr || spi_config == nullptr) {
    return false;
  }

  // Validate panel type
  if (config->panel ==
      wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_UNSPECIFIED) {
    return false; // Unsupported panel type
  }

  // Validate mode is a correct EPD mode
  if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_UNSPECIFIED) {
    return false; // Unsupported mode
  }

  // If we already have a display driver assigned to this hardware instance,
  // clean it up!
  if (_thinkink_driver ==
      wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_290_GRAYSCALE4_MFGN) {
    delete _disp_thinkink_grayscale4_eaamfgn;
    _disp_thinkink_grayscale4_eaamfgn = nullptr;
    _thinkink_driver =
        wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_UNSPECIFIED;
  } else if (
      _thinkink_driver ==
      wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_290_GRAYSCALE4_T5) {
    delete _disp_thinkink_grayscale4_t5;
    _disp_thinkink_grayscale4_t5 = nullptr;
    _thinkink_driver =
        wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_UNSPECIFIED;
  }

  // Parse all SPI bus pins
  // Check length
  if (strlen(spi_config->pin_dc) < 2 || strlen(spi_config->pin_rst) < 2 ||
      strlen(spi_config->pin_cs) < 2) {
    return false;
  }
  // SPI pins must start with 'D'
  if (spi_config->pin_dc[0] != 'D' || spi_config->pin_rst[0] != 'D' ||
      spi_config->pin_cs[0] != 'D') {
    return false;
  }

  // Parse and assign pins
  int16_t srcs = -1, busy = -1;
  int16_t dc = (int16_t)atoi(spi_config->pin_dc + 1);
  int16_t rst = (int16_t)atoi(spi_config->pin_rst + 1);
  int16_t cs = (int16_t)atoi(spi_config->pin_cs + 1);

  // Optionally parse SRAM CS and BUSY pins
  if (strlen(spi_config->pin_sram_cs) >= 2 &&
      spi_config->pin_sram_cs[0] == 'D') {
    srcs = (int16_t)atoi(spi_config->pin_sram_cs + 1);
  }
  if (strlen(spi_config->pin_busy) >= 2 && spi_config->pin_busy[0] == 'D') {
    busy = (int16_t)atoi(spi_config->pin_busy + 1);
  }

  // TODO: Configure SPI bus selection (UNUSED AS OF RIGHT NOW)

  // Configure EPD mode
  thinkinkmode_t epd_mode;
  if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_GRAYSCALE4) {
    epd_mode = THINKINK_GRAYSCALE4;
  } else if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_MONO) {
    epd_mode = THINKINK_MONO;
  }

  // Assign driver instance based on panel type
  if (config->panel ==
      wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_290_GRAYSCALE4_MFGN) {
    _disp_thinkink_grayscale4_eaamfgn =
        new ThinkInk_290_Grayscale4_EAAMFGN(dc, rst, cs, srcs, busy);
    if (!_disp_thinkink_grayscale4_eaamfgn)
      return false; // Allocation failed
    // Initialize the display
    _disp_thinkink_grayscale4_eaamfgn->begin(epd_mode);
    _thinkink_driver =
        wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_290_GRAYSCALE4_MFGN;
  } else if (
      config->panel ==
      wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_290_GRAYSCALE4_T5) {
    _disp_thinkink_grayscale4_t5 =
        new ThinkInk_290_Grayscale4_T5(dc, rst, cs, srcs, busy);
    if (!_disp_thinkink_grayscale4_t5)
      return false; // Allocation failed
    // Initialize the display
    _disp_thinkink_grayscale4_t5->begin(epd_mode);
    _thinkink_driver =
        wippersnapper_display_v1_EPDThinkInkPanel_EPD_THINK_INK_PANEL_290_GRAYSCALE4_T5;
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
