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

/*!
    @brief  Constructs a DisplayHardware with no active driver.
*/
DisplayHardware::DisplayHardware() { memset(_name, 0, sizeof(_name)); }

/*!
    @brief  Destructor. Tears down the underlying display driver, if any.
*/
DisplayHardware::~DisplayHardware() {
  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }
}

/*!
    @brief  Returns the name of this display instance.
    @return Pointer to the display's name string.
*/
const char *DisplayHardware::getName() { return _name; }

/*!
    @brief  Parses a "D<n>" pin string into an integer pin number.
    @param  pinStr  Null-terminated pin string (e.g. "D5").
    @return The parsed pin number, or -1 if the input is invalid.
*/
int16_t DisplayHardware::parsePin(const char *pinStr) {
  if (!pinStr || strlen(pinStr) < 2 || pinStr[0] != 'D') {
    return -1;
  }
  return atoi(pinStr + 1);
}

/*!
    @brief  Initializes the underlying display driver from a Display Add
            message.
    @param  addMsg  The Display Add message describing the display.
    @param  name    The unique name for this display instance.
    @return True if initialization succeeded, False otherwise.
*/
bool DisplayHardware::begin(ws_display_Add *addMsg, const char *name) {
  if (!addMsg) {
    WS_DEBUG_PRINTLN("[display] ERROR: Null add message!");
    return false;
  }

  snprintf(_name, sizeof(_name), "%s", name ? name : "");
  _class = addMsg->type;

  WS_DEBUG_PRINT("[display] Type: ");
  WS_DEBUG_PRINTLNVAR(_class);
  WS_DEBUG_PRINT("[display] Driver: ");
  WS_DEBUG_PRINTLNVAR(addMsg->driver);
  WS_DEBUG_PRINT("[display] Panel: ");
  WS_DEBUG_PRINTLNVAR(addMsg->panel);

  switch (addMsg->which_interface_type) {
  // DSI + i8080 todo
  case ws_display_Add_spi_tft_tag:
    return beginSpiTft(addMsg);
  case ws_display_Add_spi_epd_tag:
    return beginSpiEpd(addMsg);
  case ws_display_Add_ttl_rgb666_tag:
    return beginTtlRgb666(addMsg);
  case ws_display_Add_i2c_tag:
    return beginI2cDisplay(addMsg);
  default:
    WS_DEBUG_PRINTLN("[display] ERROR: Unsupported display interface type!");
    return false;
  }
}

/*!
    @brief  Initializes an SPI TFT display driver from an Add message.
    @param  msg  The Display Add message with SPI TFT interface and config.
    @return True on success, False on failure.
*/
bool DisplayHardware::beginSpiTft(ws_display_Add *msg) {
  ws_display_TftSpiConfig *spi = &msg->interface_type.spi_tft;
  ws_spi_Config *dev = &spi->spi;

  int16_t cs = parsePin(dev->pin_cs);
  int16_t dc = parsePin(spi->pin_dc);
  int16_t mosi = parsePin(dev->pin_mosi);
  int16_t sck = parsePin(dev->pin_sck);

  int16_t rst = parsePin(spi->pin_rst);
  int16_t miso = parsePin(dev->pin_miso);

  WS_DEBUG_PRINT("[display] SPI TFT pins - CS:");
  WS_DEBUG_PRINTVAR(cs);
  WS_DEBUG_PRINT(" DC:");
  WS_DEBUG_PRINTVAR(dc);
  WS_DEBUG_PRINT(" MOSI:");
  WS_DEBUG_PRINTVAR(mosi);
  WS_DEBUG_PRINT(" SCK:");
  WS_DEBUG_PRINTVAR(sck);
  WS_DEBUG_PRINT(" RST:");
  WS_DEBUG_PRINTVAR(rst);
  WS_DEBUG_PRINT(" MISO:");
  WS_DEBUG_PRINTLNVAR(miso);

  if (cs < 0 || dc < 0 || mosi < 0 || sck < 0) {
    WS_DEBUG_PRINT("[display] ERROR: Invalid SPI TFT pin configuration! ");
    return false;
  }

  if (dev->bus != 0) {
    // TODO: fire this back up via errorController
    WS_DEBUG_PRINTLN("[display] ERROR: Non-default SPI bus not supported!");
    return false;
  }

  if (msg->which_config != ws_display_Add_config_display_tag) {
    WS_DEBUG_PRINTLN("[display] ERROR: Expected display config for SPI TFT!");
    return false;
  }
  ws_display_DisplayProperties *config = &msg->config.config_display;

  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  if (strcmp(msg->driver, "ST7789") == 0) {
    _drvDisp = new dispDrvSt7789(cs, dc, mosi, sck, rst, miso);
  } else {
    WS_DEBUG_PRINT("[display] ERROR: Unsupported TFT driver: ");
    WS_DEBUG_PRINTLNVAR(msg->driver);
    return false;
  }

  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to allocate TFT driver!");
    return false;
  }

#if defined(TFT_BACKLITE)
  _drvDisp->setBacklightPin(TFT_BACKLITE);
#elif defined(TFT_BACKLIGHT)
  _drvDisp->setBacklightPin(TFT_BACKLIGHT);
#endif

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

// ---------------------------------------------------------------------------
// Bit-bang SPI helper for EPD auto-detection
// ---------------------------------------------------------------------------
/*!
    @brief  Bit-bangs an SPI read of a single register on an EPD before any
            hardware SPI driver is bound, used to probe the panel for
            auto-detection.
    @param  cmd     The command/register byte to issue.
    @param  config  EPD SPI pin configuration to use for the probe.
    @return The byte returned by the panel.
*/
uint8_t
DisplayHardware::EpdBitBangReadRegister(uint8_t cmd,
                                        ws_display_EpdSpiConfig *config) {
  int16_t mosi = DisplayHardware::parsePin(config->spi.pin_mosi);
  int16_t miso = DisplayHardware::parsePin(config->spi.pin_miso);
  int16_t sck = DisplayHardware::parsePin(config->spi.pin_sck);
  int16_t cs = DisplayHardware::parsePin(config->spi.pin_cs);
  int16_t dc = DisplayHardware::parsePin(config->pin_dc);
  int16_t rst = DisplayHardware::parsePin(config->pin_rst);

  pinMode(mosi, OUTPUT);
  pinMode(sck, OUTPUT);
  pinMode(cs, OUTPUT);
  pinMode(dc, OUTPUT);
  pinMode(rst, OUTPUT);

  // Reset the display
  digitalWrite(cs, HIGH);
  digitalWrite(rst, HIGH);
  delay(10);
  digitalWrite(rst, LOW);
  delay(10);
  digitalWrite(rst, HIGH);
  delay(200);

  // Begin transaction
  digitalWrite(cs, LOW);
  digitalWrite(dc, LOW);
  digitalWrite(mosi, LOW);
  digitalWrite(rst, HIGH);
  digitalWrite(sck, LOW);

  // Write command
  for (int i = 0; i < 8; i++) {
    digitalWrite(mosi, (cmd & (1 << (7 - i))) != 0);
    digitalWrite(sck, HIGH);
    digitalWrite(sck, LOW);
  }

  // Switch read pin to input
  pinMode(miso, INPUT_PULLUP);
  digitalWrite(dc, HIGH);

  // Read response
  uint8_t status = 0;
  for (int i = 0; i < 8; i++) {
    status <<= 1;
    if (digitalRead(miso))
      status |= 1;
    digitalWrite(sck, HIGH);
    digitalWrite(sck, LOW);
  }

  digitalWrite(cs, HIGH);
  pinMode(mosi, OUTPUT);
  return status;
}

/*!
    @brief  Probes for an SSD1680 EPD controller.
    @param  config  EPD SPI pin configuration to probe.
    @return True if the panel responds as an SSD1680.
*/
bool DisplayHardware::detect_ssd1680(ws_display_EpdSpiConfig *config) {
  uint8_t status = EpdBitBangReadRegister(0x71, config);
  return status == 0xFF;
}

/*!
    @brief  Probes for an SSD1683 EPD controller.
    @param  config  EPD SPI pin configuration to probe.
    @return True if the panel responds as an SSD1683.
*/
bool DisplayHardware::detect_ssd1683(ws_display_EpdSpiConfig *config) {
  uint8_t status = EpdBitBangReadRegister(0x2F, config);
  return (status & 0x03) == 0x01;
}

/*!
    @brief  Probes for a UC8151D EPD controller.
    @param  config  EPD SPI pin configuration to probe.
    @return True if the panel responds as a UC8151D.
*/
bool DisplayHardware::detect_uc8151d(ws_display_EpdSpiConfig *config) {
  uint8_t rev = EpdBitBangReadRegister(0x70, config);
  return rev != 0xFF;
}

/*!
    @brief  Probes for a UC8179 EPD controller.
    @param  config  EPD SPI pin configuration to probe.
    @return True if the panel responds as a UC8179.
*/
bool DisplayHardware::detect_uc8179(ws_display_EpdSpiConfig *config) {
  uint8_t dualspi = EpdBitBangReadRegister(0x15, config);
  return dualspi != 0xFF;
}

/*!
    @brief  Probes for a UC8253 EPD controller.
    @param  config  EPD SPI pin configuration to probe.
    @return True if the panel responds as a UC8253.
*/
bool DisplayHardware::detect_uc8253(ws_display_EpdSpiConfig *config) {
  uint8_t status = EpdBitBangReadRegister(0x71, config);
  return status != 0xFF;
}

// ---------------------------------------------------------------------------
// SPI EPD initialization
// ---------------------------------------------------------------------------
/*!
    @brief  Initializes an SPI eInk display driver from an Add message,
            including MagTag auto-detection of the underlying panel.
    @param  msg  The Display Add message with SPI EPD interface and config.
    @return True on success, False on failure.
*/
bool DisplayHardware::beginSpiEpd(ws_display_Add *msg) {
  ws_display_EpdSpiConfig *spi_epd_config = &msg->interface_type.spi_epd;
  ws_spi_Config *spi_pin_config = &spi_epd_config->spi;

  int16_t dc = parsePin(spi_epd_config->pin_dc);
  int16_t rst = parsePin(spi_epd_config->pin_rst);
  int16_t cs = parsePin(spi_pin_config->pin_cs);
  int16_t mosi = -1;
  int16_t sck = -1;
  int16_t miso = -1;
  int16_t sram_cs = -1, busy = -1;
  if (strlen(spi_pin_config->pin_mosi) >= 2)
    mosi = parsePin(spi_pin_config->pin_mosi);
  if (strlen(spi_pin_config->pin_sck) >= 2)
    sck = parsePin(spi_pin_config->pin_sck);
  if (strlen(spi_pin_config->pin_miso) >= 2)
    miso = parsePin(spi_pin_config->pin_miso);
  if (strlen(spi_epd_config->pin_sram_cs) >= 2)
    sram_cs = parsePin(spi_epd_config->pin_sram_cs);
  if (strlen(spi_epd_config->pin_busy) >= 2)
    busy = parsePin(spi_epd_config->pin_busy);

  WS_DEBUG_PRINT("[display] SPI EPD pins - DC:");
  WS_DEBUG_PRINTVAR(dc);
  WS_DEBUG_PRINT(" RST:");
  WS_DEBUG_PRINTVAR(rst);
  WS_DEBUG_PRINT(" CS:");
  WS_DEBUG_PRINTVAR(cs);
  WS_DEBUG_PRINT(" MOSI:");
  WS_DEBUG_PRINTVAR(mosi);
  WS_DEBUG_PRINT(" SCK:");
  WS_DEBUG_PRINTVAR(sck);
  WS_DEBUG_PRINT(" MISO:");
  WS_DEBUG_PRINTVAR(miso);
  WS_DEBUG_PRINT(" SRAM_CS:");
  WS_DEBUG_PRINTVAR(sram_cs);
  WS_DEBUG_PRINT(" BUSY:");
  WS_DEBUG_PRINTLNVAR(busy);

  // EPD drivers currently use the default hardware SPI bus/pins.
  // Reject explicit non-default SPI pins from the payload.
  if ((mosi >= 0 && mosi != MOSI) || (sck >= 0 && sck != SCK) ||
      (miso >= 0 && miso != MISO)) {
    WS_DEBUG_PRINTLN(
        "[display] ERROR: SPI EPD only supports default MOSI/SCK/MISO pins!");
    return false;
  }

  if (spi_pin_config->bus != 0) {
    WS_DEBUG_PRINTLN("[display] ERROR: Non-default SPI bus not supported!");
    return false;
  }

  if (msg->which_config != ws_display_Add_config_epd_tag) {
    WS_DEBUG_PRINTLN("[display] ERROR: Expected EPD config for SPI EPD!");
    return false;
  }
  ws_display_EPDConfig *config = &msg->config.config_epd;

  // Parse EPD mode
  if (config->mode == ws_display_EPDMode_EPD_MODE_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[display] ERROR: EPD mode is unspecified!");
    return false;
  }

  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  // MagTag hardware auto-detection (requires SPI probing)
  const char *driver = msg->driver;
  if (strncmp(_name, "eink-magtag", sizeof("eink-magtag") - 1) == 0 &&
      strlen(driver) == 0) {
    if (EpdBitBangReadRegister(0x71, spi_epd_config) == 0xFF) {
      driver = "SSD1680";
    } else {
      driver = "ILI0373";
    }
    WS_DEBUG_PRINT("[display] MagTag auto-detected driver: ");
    WS_DEBUG_PRINTLNVAR(driver);
  }

  // Create driver based on driver string (component-name resolution
  // already done by controller)
  if (strcmp(driver, "SSD1680") == 0) {
    _drvDisp = new drvDispThinkInkGrayscale4Eaamfgn(dc, rst, cs, sram_cs, busy);
  } else if (strcmp(driver, "ILI0373") == 0) {
    _drvDisp = new dispDrvThinkInkGrayscale4T5(dc, rst, cs, sram_cs, busy);
  } else if (strcmp(driver, "SSD1683") == 0) {
    _drvDisp = new dispDrvThinkInkGrayscale4MFGN(dc, rst, cs, sram_cs, busy);
  } else if (strcmp(driver, "UC8179") == 0) {
    _drvDisp = new dispDrvThinkInkMonoAAAMFGN(dc, rst, cs, sram_cs, busy);
  } else if (strcmp(driver, "UC8253") == 0) {
    _drvDisp = new dispDrvThinkInkMonoBAAMFGN(dc, rst, cs, sram_cs, busy);
  } else if (strcmp(driver, "UC8151") == 0) {
    _drvDisp = new dispDrvThinkInkMonoM06(dc, rst, cs, sram_cs, busy);
  } else {
    WS_DEBUG_PRINT("[display] ERROR: Unsupported EPD driver: ");
    WS_DEBUG_PRINTLNVAR(driver);
    return false;
  }

  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to allocate EPD driver!");
    return false;
  }

  thinkinkmode_t epd_mode;
  if (config->mode == ws_display_EPDMode_EPD_MODE_GRAYSCALE4)
    epd_mode = THINKINK_GRAYSCALE4;
  else
    epd_mode = THINKINK_MONO;

  _drvDisp->setWidth(config->properties.width);
  _drvDisp->setHeight(config->properties.height);
  if (config->properties.text_size > 0)
    _drvDisp->setTextSize(config->properties.text_size);

  if (!_drvDisp->begin(epd_mode)) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to begin EPD driver!");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  WS_DEBUG_PRINTLN("[display] SPI EPD initialized successfully!");
  return true;
}

// ---------------------------------------------------------------------------
// TTL RGB666 initialization (Qualia ESP32-S3)
// ---------------------------------------------------------------------------
/*!
    @brief  Initializes a TTL RGB666 panel driver (Qualia ESP32-S3 only).
    @param  msg  The Display Add message with RGB666 interface and config.
    @return True on success, False on failure or when unsupported on this
            board.
*/
bool DisplayHardware::beginTtlRgb666(ws_display_Add *msg) {
#ifdef ARDUINO_ADAFRUIT_QUALIA_S3_RGB666
  if (msg->which_config != ws_display_Add_config_display_tag) {
    WS_DEBUG_PRINTLN("[display] ERROR: Expected display config for RGB666!");
    return false;
  }
  ws_display_DisplayProperties *config = &msg->config.config_display;

  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  WS_DEBUG_PRINT("[display] RGB666 panel: ");
  WS_DEBUG_PRINTLNVAR(msg->panel);

  _drvDisp = new dispDrvRgb666(msg->panel);
  if (!_drvDisp)
    return false;

  int16_t width = config->width;
  int16_t height = config->height;

  // No controller payloads may omit RGB666 dimensions.
  if (width <= 0 || height <= 0) {
    WS_DEBUG_PRINTLN("[display] ERROR: RGB666 config missing dimensions, "
                     "failed to initialise display.");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  _drvDisp->setWidth(width);
  _drvDisp->setHeight(height);
  _drvDisp->setRotation(config->rotation);
  if (config->text_size > 0)
    _drvDisp->setTextSize(config->text_size);

  if (!_drvDisp->begin()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to begin RGB666 driver!");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  WS_DEBUG_PRINTLN("[display] TTL RGB666 initialized successfully!");
  return true;
#else
  WS_DEBUG_PRINTLN("[display] ERROR: TTL RGB666 not supported on this board!");
  return false;
#endif
}

// ---------------------------------------------------------------------------
// I2C display initialization (OLED, CharLCD, LED backpack, etc.)
// ---------------------------------------------------------------------------
/*!
    @brief  Initializes an I2C display driver (OLED, CharLCD, LED backpack)
            from an Add message.
    @param  msg  The Display Add message with I2C interface and config.
    @return True on success, False on failure.
*/
bool DisplayHardware::beginI2cDisplay(ws_display_Add *msg) {
  if (!msg) {
    WS_DEBUG_PRINTLN("[display] ERROR: Null add message for I2C display!");
    return false;
  }

  if (msg->which_interface_type != ws_display_Add_i2c_tag) {
    WS_DEBUG_PRINTLN(
        "[display] ERROR: Expected I2C interface for I2C display!");
    return false;
  }
  ws_i2c_DeviceDescriptor *i2c_cfg = &msg->interface_type.i2c;
  uint16_t addr = (uint16_t)i2c_cfg->device_address;

  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  WS_DEBUG_PRINT("[display] I2C driver: ");
  WS_DEBUG_PRINTVAR(msg->driver);
  WS_DEBUG_PRINT(" addr: 0x");
  WS_DEBUG_PRINTLNVAR(addr, HEX);

  TwoWire *i2c =
      Ws._i2c_controller->GetOrCreateI2cBus(i2c_cfg->pin_scl, i2c_cfg->pin_sda);
  if (i2c == nullptr) {
    WS_DEBUG_PRINTLN("[display] ERROR: I2C bus not found for Display device!");
    return false;
  }

  // Create the appropriate I2C display driver based on driver string
  dispDrvBaseI2c *drv = nullptr;
  const char *driverName = msg->driver;
  if (strcasecmp(driverName, "SSD1306") == 0) {
    drv = new dispDrvSsd1306(i2c, addr, 0, driverName);
  } else if (strcasecmp(driverName, "SH1107") == 0) {
    drv = new dispDrvSh1107(i2c, addr, 0, driverName);
  } else if (strcasecmp(driverName, "charlcd") == 0) {
    drv = new dispDrvCharLcd(i2c, addr, 0, driverName);
  } else if (strcasecmp(driverName, "7seg") == 0) {
    drv = new dispDrv7Seg(i2c, addr, 0, driverName);
  } else if (strcasecmp(driverName, "quadalphanum") == 0) {
    drv = new dispDrvQuadAlphaNum(i2c, addr, 0, driverName);
  } else {
    WS_DEBUG_PRINT("[display] ERROR: Unsupported I2C display driver: ");
    WS_DEBUG_PRINTLNVAR(driverName);
    return false;
  }

  if (!drv) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to allocate I2C display driver!");
    return false;
  }

  // Configure based on config type
  pb_size_t config = msg->which_config;
  WS_DEBUG_PRINT("[display] I2C config tag: ");
  WS_DEBUG_PRINTLNVAR(config);
  if (config == ws_display_Add_config_display_tag) {
    ws_display_DisplayProperties *cfg = &msg->config.config_display;
    WS_DEBUG_PRINT("[display] OLED config: ");
    WS_DEBUG_PRINTVAR(cfg->width);
    WS_DEBUG_PRINT("x");
    WS_DEBUG_PRINTLNVAR(cfg->height);
    drv->ConfigureSSD1306(cfg->width, cfg->height,
                          cfg->text_size > 0 ? cfg->text_size : 1);
  } else if (config == ws_display_Add_config_char_lcd_tag) {
    ws_display_CharLcdConfig *cfg = &msg->config.config_char_lcd;
    drv->ConfigureCharLcd(cfg->rows, cfg->columns, true);
  } else if (config == ws_display_Add_config_led_tag) {
    ws_display_LedBackpackConfig *cfg = &msg->config.config_led;
    drv->ConfigureI2CBackpack(cfg->brightness, cfg->alignment);
  } else {
    // No matching config — for OLEDs, apply safe defaults to prevent
    // crash from uninitialized width/height in dispDrvSsd1306::begin()
    WS_DEBUG_PRINTLN("[display] WARNING: No config for I2C display, "
                     "applying defaults");
    if (strcasecmp(driverName, "SSD1306") == 0) {
      drv->ConfigureSSD1306(128, 32, 1);
    } else if (strcasecmp(driverName, "SH1107") == 0) {
      drv->ConfigureSSD1306(128, 64, 1);
    }
  }

  if (!drv->begin()) {
    WS_DEBUG_PRINTLN("[display] ERROR: Failed to begin I2C display driver!");
    delete drv;
    return false;
  }

  _drvDisp = drv;
  WS_DEBUG_PRINTLN("[display] I2C display initialized successfully!");
  return true;
}

/*!
    @brief  Shows the driver's splash screen, if supported.
*/
void DisplayHardware::showSplash() {
  if (_drvDisp)
    _drvDisp->showSplash();
}

/*!
    @brief  Draws the status bar and the Adafruit IO username.
    @param  io_username  Adafruit IO username to display.
*/
void DisplayHardware::drawStatusBar(const char *io_username) {
  if (_drvDisp)
    _drvDisp->drawStatusBar(io_username);
}

/*!
    @brief  Updates status bar icons based on current state.
    @param  rssi            Current WiFi RSSI value.
    @param  bat             Current battery level (0-100).
    @param  mqtt_connected  True if MQTT is connected.
*/
void DisplayHardware::updateStatusBar(int8_t rssi, uint8_t bat,
                                      bool mqtt_connected) {
  if (_drvDisp)
    _drvDisp->updateStatusBar(rssi, bat, mqtt_connected);
}

/*!
    @brief  Writes a Display Write message to the display.
    @param  msg  The Display Write message to render.
    @return True if the write succeeded, False otherwise.
*/
bool DisplayHardware::write(ws_display_Write *msg) {
  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] ERROR: No display driver initialized!");
    return false;
  }

  WS_DEBUG_PRINT("[display] Writing message: ");
  WS_DEBUG_PRINTLNVAR(msg->message);
  _drvDisp->writeMessage(msg->message);
  return true;
}
