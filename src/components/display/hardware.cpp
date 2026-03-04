/*!
 * @file src/components/display/hardware.cpp
 *
 * Implementation for the display hardware (V2).
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

DisplayHardware::DisplayHardware() {
  memset(_name, 0, sizeof(_name));
  _type = ws_display_DisplayType_DISPLAY_TYPE_UNSPECIFIED;
}

DisplayHardware::~DisplayHardware() {
  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }
}

/*!
    @brief  Gets the display's name.
    @return The display name string.
*/
const char *DisplayHardware::getName() { return _name; }

/*!
    @brief  Parses a pin string (e.g., "D5") and returns the pin number.
    @param  pinStr  The pin string to parse.
    @return The pin number, or -1 if invalid.
*/
int16_t DisplayHardware::parsePin(const char *pinStr) {
  if (!pinStr || strlen(pinStr) < 2 || pinStr[0] != 'D') {
    return -1;
  }
  return atoi(pinStr + 1);
}

/*!
    @brief  Initializes the display from an Add message.
    @param  addMsg  Pointer to the ws_display_Add message.
    @return True if initialization was successful, False otherwise.
*/
bool DisplayHardware::begin(ws_display_Add *addMsg) {
  // Store name and type
  strncpy(_name, addMsg->name, sizeof(_name) - 1);
  _name[sizeof(_name) - 1] = '\0';
  _type = addMsg->type;

  WS_DEBUG_PRINT("[display] Type: ");
  WS_DEBUG_PRINTLN(_type);
  WS_DEBUG_PRINT("[display] Driver: ");
  WS_DEBUG_PRINTLN(addMsg->driver);
  WS_DEBUG_PRINT("[display] Panel: ");
  WS_DEBUG_PRINTLN(addMsg->panel);

  // Route to appropriate begin method based on interface type
  switch (addMsg->which_interface_type) {
  case ws_display_Add_spi_tft_tag:
    return beginSpiTft(addMsg);
  // Future interface types:
  // case ws_display_Add_spi_epd_tag:
  // case ws_display_Add_i2c_tag:
  // case ws_display_Add_ttl_rgb666_tag:
  // case ws_display_Add_i8080_tag:
  // case ws_display_Add_dsi_tag:
  default:
    WS_DEBUG_PRINTLN(
        "[display] ERROR: Unsupported display interface type!");
    return false;
  }
}

/*!
    @brief  Initializes an SPI TFT display.
    @param  msg  Pointer to the ws_display_Add message.
    @return True if initialization was successful, False otherwise.
*/
bool DisplayHardware::beginSpiTft(ws_display_Add *msg) {
  ws_display_TftSpiConfig *spi = &msg->interface_type.spi_tft;

  // Parse SPI pins
  int16_t cs = parsePin(spi->pin_cs);
  int16_t dc = parsePin(spi->pin_dc);
  int16_t mosi = parsePin(spi->pin_mosi);
  int16_t sck = parsePin(spi->pin_sck);
  int16_t rst = parsePin(spi->pin_rst);
  int16_t miso = parsePin(spi->pin_miso);

  WS_DEBUG_PRINT("[display] SPI TFT pins - CS:");
  WS_DEBUG_PRINT(cs);
  WS_DEBUG_PRINT(" DC:");
  WS_DEBUG_PRINT(dc);
  WS_DEBUG_PRINT(" MOSI:");
  WS_DEBUG_PRINT(mosi);
  WS_DEBUG_PRINT(" SCK:");
  WS_DEBUG_PRINT(sck);
  WS_DEBUG_PRINT(" RST:");
  WS_DEBUG_PRINT(rst);
  WS_DEBUG_PRINT(" MISO:");
  WS_DEBUG_PRINTLN(miso);

  // Non-default SPI bus not supported yet
  if (spi->bus != 0) {
    WS_DEBUG_PRINTLN("[display] ERROR: Non-default SPI bus not supported!");
    return false;
  }

  // Get config
  if (msg->which_config != ws_display_Add_config_tft_tag) {
    WS_DEBUG_PRINTLN("[display] ERROR: Expected TFT config for SPI TFT!");
    return false;
  }
  ws_display_TftConfig *config = &msg->config.config_tft;

  // Clean up existing driver
  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  // Create driver based on driver string
  if (strcmp(msg->driver, "ST7789") == 0) {
    _drvDisp = new dispDrvSt7789(cs, dc, mosi, sck, rst, miso);
  } else {
    WS_DEBUG_PRINT("[display] ERROR: Unsupported TFT driver: ");
    WS_DEBUG_PRINTLN(msg->driver);
    return false;
  }

  // TODO: Move backlight pin into proto Add message instead of board defines
#if defined(TFT_BACKLITE)
  _drvDisp->setBacklightPin(TFT_BACKLITE);
#elif defined(TFT_BACKLIGHT)
  _drvDisp->setBacklightPin(TFT_BACKLIGHT);
#endif

  // Configure dimensions and begin
  _drvDisp->setWidth(config->width);
  _drvDisp->setHeight(config->height);
  _drvDisp->setRotation(config->rotation);
  if (config->text_size > 0) {
    _drvDisp->setTextSize(config->text_size);
  }

  if (!_drvDisp->begin()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to begin TFT driver!");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  WS_DEBUG_PRINTLN("[display] SPI TFT initialized successfully!");
  return true;
}

void DisplayHardware::showSplash() {
  if (_drvDisp)
    _drvDisp->showSplash();
}

void DisplayHardware::drawStatusBar(const char *io_username) {
  if (_drvDisp)
    _drvDisp->drawStatusBar(io_username);
}

void DisplayHardware::updateStatusBar(int8_t rssi, uint8_t bat,
                                      bool mqtt_connected) {
  if (_drvDisp)
    _drvDisp->updateStatusBar(rssi, bat, mqtt_connected);
}

/*!
    @brief  Writes content to the display.
    @param  msg  Pointer to the ws_display_Write message.
    @return True if successful, False otherwise.
*/
bool DisplayHardware::write(ws_display_Write *msg) {
  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] ERROR: No display driver initialized!");
    return false;
  }

  switch (msg->which_content) {
  case ws_display_Write_message_tag:
    WS_DEBUG_PRINT("[display] Writing message: ");
    WS_DEBUG_PRINTLN(msg->content.message);
    _drvDisp->writeMessage(msg->content.message, msg->clear_first,
                           msg->cursor_x, msg->cursor_y);
    return true;
  // Future content types:
  // case ws_display_Write_url_tag:
  // case ws_display_Write_base64image_tag:
  // case ws_display_Write_binary_image_tag:
  default:
    WS_DEBUG_PRINTLN("[display] ERROR: Unsupported write content type!");
    return false;
  }
}
