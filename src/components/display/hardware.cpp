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
    @brief  Lambda function to create a dispDrvBase instance
*/
using FnCreateDispDrv =
    std::function<dispDrvBase *(int16_t, int16_t, int16_t, int16_t, int16_t)>;

// Factory for creating a new display drivers
// NOTE: When you add a new display driver, make sure to add it to the factory!
static const std::map<std::string, FnCreateDispDrv> FactoryDrvDisp = {
    {"grayscale4_eaamfgn",
     [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
        int16_t busy) -> dispDrvBase * {
       return new drvDispThinkInkGrayscale4Eaamfgn(dc, rst, cs, sram_cs, busy);
     }},
    {"magtag_2025",
     [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
        int16_t busy) -> dispDrvBase * {
       return new drvDispThinkInkGrayscale4Eaamfgn(dc, rst, cs, sram_cs, busy);
     }}};

/*!
    @brief  Creates a new display driver instance based on the driver name.
    @param  driver_name
            The name of the display driver to create.
    @param  dc
            Data/Command pin number.
    @param  rst
            Reset pin number.
    @param  cs
            Chip Select pin number.
    @param  sram_cs
            Optional SRAM Chip Select pin number (default: -1).
    @param  busy
            Optional Busy pin number (default: -1).
    @return Pointer to the created display driver instance, or nullptr if the
            driver name is not recognized.
*/
dispDrvBase *CreateDrvDisp(const char *driver_name, int16_t dc, int16_t rst,
                           int16_t cs, int16_t sram_cs = -1,
                           int16_t busy = -1) {
  auto it = FactoryDrvDisp.find(driver_name);
  if (it == FactoryDrvDisp.end())
    return nullptr;

  return it->second(dc, rst, cs, sram_cs, busy);
}

/*!
    @brief  Constructs a new DisplayHardware object
    @param  name
            The name of the hardware instance.
*/
DisplayHardware::DisplayHardware(const char *name) {
  strncpy(_name, name, sizeof(_name) - 1);
  _name[sizeof(_name) - 1] = '\0';
  _type = wippersnapper_display_v1_DisplayType_DISPLAY_TYPE_UNSPECIFIED;
}

/*!
    @brief  Destructor
*/
DisplayHardware::~DisplayHardware() {
  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }
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
    WS_DEBUG_PRINTLN("[display] EPD config or SPI config is null!");
    return false;
  }

  // Validate mode is a correct EPD mode
  if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[display] Unsupported EPD mode!");
    return false; // Unsupported mode
  }

  // TODO: If we already have a display driver assigned to this hardware
  // instance, clean it up!

  // Parse all SPI bus pins
  // Check length
  if (strlen(spi_config->pin_dc) < 2 || strlen(spi_config->pin_rst) < 2 ||
      strlen(spi_config->pin_cs) < 2) {
    WS_DEBUG_PRINTLN("[display] Invalid SPI pin len!");
    return false;
  }
  // SPI pins must start with 'D'
  if (spi_config->pin_dc[0] != 'D' || spi_config->pin_rst[0] != 'D' ||
      spi_config->pin_cs[0] != 'D') {
    WS_DEBUG_PRINTLN("[display] SPI pins must start with 'D'!");
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

  // Create display driver object using the factory function
  _drvDisp = CreateDrvDisp(_name, dc, rst, cs, srcs, busy);
  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] Failed to create display driver!");
    return false; // Failed to create display driver
  }

  // Configure EPD mode
  thinkinkmode_t epd_mode = THINKINK_MONO;
  if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_GRAYSCALE4) {
    epd_mode = THINKINK_GRAYSCALE4;
    WS_DEBUG_PRINTLN("[display] EPD mode: GRAYSCALE4");
  }

  if (!_drvDisp->begin(epd_mode)) {
    WS_DEBUG_PRINTLN("[display] Failed to begin display driver!");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  return true; // Configuration successful
}

/*!
    @brief  Gets the name of the display hardware instance.
    @return The name of the display hardware instance.
*/
const char *DisplayHardware::getName() { return _name; }

/*!
    @brief  Writes a message to the display.
    @param  message
            The message to display.
*/
void DisplayHardware::writeMessage(const char *message) {
  if (_drvDisp) {
    _drvDisp->writeMessage(message);
  } else {
    WS_DEBUG_PRINTLN("[display] No display driver initialized!");
  }
}