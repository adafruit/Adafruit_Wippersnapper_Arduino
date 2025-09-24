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
    @brief  Lambda function to create a dispDrvBase EPD instance.
*/
using FnCreateDispDrvEpd =
    std::function<dispDrvBase *(int16_t, int16_t, int16_t, int16_t, int16_t)>;

// Factory for creating a new display drivers
// NOTE: When you add a new display driver, make sure to add it to the factory!
static const std::map<std::string, FnCreateDispDrvEpd> FactoryDrvDispEpd = {
    {"thinkink-gs4-eaamfgn",
     [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
        int16_t busy) -> dispDrvBase * {
       return new drvDispThinkInkGrayscale4Eaamfgn(dc, rst, cs, sram_cs, busy);
     }},
    {"thinkink-gs4-t5",
     [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
        int16_t busy) -> dispDrvBase * {
       return new dispDrvThinkInkGrayscale4T5(dc, rst, cs, sram_cs, busy);
     }}};

/*!
    @brief  Lambda function to create a dispDrvBase SPI TFT instance
*/
using FnCreateDispDrvTft = std::function<dispDrvBase *(
    int16_t, int16_t, int16_t, int16_t, int16_t, int16_t)>;

// Factory for creating a new SPI TFT display driver
// NOTE: When you add a new SPI TFT display driver, make sure to add it to the
// factory!
static const std::map<std::string, FnCreateDispDrvTft> FactoryDrvDispTft = {
    {"st7735",
     [](int16_t cs, int16_t dc, int16_t mosi, int16_t sck, int16_t rst,
        int16_t miso) -> dispDrvBase * {
       return new dispDrvSt7789(cs, dc, mosi, sck, rst, miso);
     }},
    {"st7789",
     [](int16_t cs, int16_t dc, int16_t mosi, int16_t sck, int16_t rst,
        int16_t miso) -> dispDrvBase * {
       return new dispDrvSt7789(cs, dc, mosi, sck, rst, miso);
     }}};

/*!
    @brief  Creates a new E-Ink display driver instance based on the driver
   name.
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
dispDrvBase *CreateDrvDispEpd(const char *driver_name, int16_t dc, int16_t rst,
                              int16_t cs, int16_t sram_cs = -1,
                              int16_t busy = -1) {
  auto it = FactoryDrvDispEpd.find(driver_name);
  if (it == FactoryDrvDispEpd.end())
    return nullptr;

  return it->second(dc, rst, cs, sram_cs, busy);
}

/*!
    @brief  Creates a new SPI TFT display driver instance based on the driver
   name.
    @param  driver_name
            The name of the SPI TFT display driver to create.
    @param  cs
            Chip Select pin number.
    @param  dc
            Data/Command pin number.
    @param  mosi
            MOSI pin number.
    @param  sck
            SCK pin number.
    @param  rst
            Optional Reset pin number (default: -1).
    @param  miso
            Optional MISO pin number (default: -1).
    @return Pointer to the created display driver instance, or nullptr if the
            driver name is not recognized.
*/
dispDrvBase *CreateDrvDispTft(const char *driver_name, int16_t cs, int16_t dc,
                              int16_t mosi, int16_t sck, int16_t rst = -1,
                              int16_t miso = -1) {
  auto it = FactoryDrvDispTft.find(driver_name);
  if (it == FactoryDrvDispTft.end())
    return nullptr;

  return it->second(cs, dc, mosi, sck, rst, miso);
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
    @brief  Parses a pin string (e.g., "D5") and returns the corresponding pin
   number.
    @param  pinStr
            The pin string to parse.
    @return The pin number, or -1 if the string is invalid.
*/
int16_t DisplayHardware::parsePin(const char *pinStr) {
  if (!pinStr || strlen(pinStr) < 2 || pinStr[0] != 'D') {
    return -1;
  }
  return atoi(pinStr + 1);
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
    return false;
  }

  // If we already have a display driver assigned to this hardware instance,
  // clean it up!
  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  // Parse and assign pins
  int16_t srcs = -1, busy = -1;
  int16_t dc = parsePin(spi_config->pin_dc);
  int16_t rst = parsePin(spi_config->pin_rst);
  int16_t cs = parsePin(spi_config->pin_cs);

  // Optionally parse SRAM CS and BUSY pins
  if (strlen(spi_config->pin_sram_cs) >= 2) {
    srcs = parsePin(spi_config->pin_sram_cs);
  }
  if (strlen(spi_config->pin_busy) >= 2) {
    busy = parsePin(spi_config->pin_busy);
  }

  // Configure SPI bus
  if (spi_config->bus != 0) {
    WS_DEBUG_PRINTLN(
        "[display] ERROR: Non-default SPI buses are currently not supported!");
    return false;
  }

  // For "magtag" component name, attempt to autodetect the driver type
  if (strncmp(_name, "magtag", 6) == 0) {
    if (detect_ssd1680(cs, dc, rst)) {
      // Detected SSD1680, use EAAMFGN driver
      strncpy(_name, "thinkink-gs4-eaamfgn", sizeof(_name) - 1);
      _name[sizeof(_name) - 1] = '\0';
    } else {
      // Did not detect SSD1680, use IL0373 driver
      strncpy(_name, "thinkink-gs4-t5", sizeof(_name) - 1);
      _name[sizeof(_name) - 1] = '\0';
    }
  }

  // Create display driver object using the factory function
  _drvDisp = CreateDrvDispEpd(_name, dc, rst, cs, srcs, busy);
  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] Failed to create display driver!");
    return false; // Failed to create display driver
  }

  // Configure EPD mode
  thinkinkmode_t epd_mode = THINKINK_MONO;
  if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_GRAYSCALE4) {
    epd_mode = THINKINK_GRAYSCALE4;
  } else if (config->mode == wippersnapper_display_v1_EPDMode_EPD_MODE_MONO) {
    epd_mode = THINKINK_MONO;
  } else {
    WS_DEBUG_PRINTLN("[display] ERROR: Unsupported EPD mode!");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  if (!_drvDisp->begin(epd_mode)) {
    WS_DEBUG_PRINTLN("[display] Failed to begin display driver!");
    delete _drvDisp;
    _drvDisp = nullptr;
    return false;
  }

  WS_DEBUG_PRINTLN("[display] Successfully initialized epd display!");
  return true;
}

void DisplayHardware::showSplash() {
  if (!_drvDisp)
    return;
  _drvDisp->showSplash();
}

void DisplayHardware::drawStatusBar(const char *io_username) {
  if (!_drvDisp)
    return;
  _drvDisp->drawStatusBar(const char *io_username);
}

void DisplayHardware::updateStatusBar(int8_t rssi, uint8_t bat,
                                      ws_status_t mqtt_connected) {
  if (!_drvDisp)
    return;
  _drvDisp->updateStatusBar(rssi, bat, mqtt_connected);
}

/*!
    @brief  Removes a suffix from the hardware instance name, if it exists.
    @param  suffix
            The suffix to remove (e.g., "-lg", "-md", "-sm").
*/
void DisplayHardware::removeSuffix(const char *suffix) {
  char *suffix_pos = strstr(_name, suffix);
  if (suffix_pos) {
    *suffix_pos = '\0'; // Truncate string at suffix position
  }
}

/*!
    @brief  Attempts to configure and initialize a TFT display
    @param  config
            Pointer to the TFT configuration structure.
    @param  spi_config
            Pointer to the SPI configuration structure for TFT.
    @return True if configuration was successful, False otherwise.
*/
bool DisplayHardware::beginTft(
    wippersnapper_display_v1_TftConfig *config,
    wippersnapper_display_v1_TftSpiConfig *spi_config) {
  // Validate pointers
  if (config == nullptr || spi_config == nullptr) {
    WS_DEBUG_PRINTLN("[display] EPD config or SPI config is null!");
    return false;
  }

  // If we already have a display driver assigned to this hardware instance,
  // clean it up!
  if (_drvDisp) {
    delete _drvDisp;
    _drvDisp = nullptr;
  }

  int16_t rst = -1, miso = -1;
  int16_t cs = parsePin(spi_config->pin_cs);
  int16_t dc = parsePin(spi_config->pin_dc);
  int16_t mosi = parsePin(spi_config->pin_mosi);
  int16_t sck = parsePin(spi_config->pin_sck);

  // Optionally parse SRAM CS and BUSY pins
  if (strlen(spi_config->pin_rst) >= 2) {
    rst = parsePin(spi_config->pin_rst);
  }
  if (strlen(spi_config->pin_miso) >= 2) {
    miso = parsePin(spi_config->pin_miso);
  }

  // Configure text size based on suffix in driver name
  uint8_t text_sz; // Default text size
  if (strstr(_name, "-lg") != nullptr) {
    // Larger text size for displays with -lg suffix
    text_sz = 4;
    removeSuffix("-lg");
  } else if (strstr(_name, "-md") != nullptr) {
    // Larger text size for displays with -md suffix
    text_sz = 3;
    removeSuffix("-md");
  } else {
    text_sz = 1;
  }

  // Create display driver object using the factory function
  _drvDisp = CreateDrvDispTft(_name, cs, dc, mosi, sck, rst, miso);
  if (!_drvDisp) {
    WS_DEBUG_PRINTLN("[display] Failed to create display driver!");
    return false;
  }

  _drvDisp->setWidth(config->width);
  _drvDisp->setHeight(config->height);
  _drvDisp->setRotation(config->rotation);
  _drvDisp->begin();

  return true;
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
  if (!_drvDisp)
    return;
  _drvDisp->writeMessage(message);
}

/*!
    @brief  Detects if an SSD1680 EPD is connected using bit-banged SPI.
    @param  cs
            Chip Select pin number.
    @param  dc
            Data/Command pin number.
    @param  rst
            Reset pin number.
    @return True if an SSD1680 is detected, False otherwise (IL0373 or different
   EPD).
*/
bool DisplayHardware::detect_ssd1680(uint8_t cs, uint8_t dc, uint8_t rst) {
  // note: for a complete implementation reference, see
  // https://github.com/adafruit/circuitpython/commit/f4316cb2491c815b128acca47f1bb75519fe306e
  // Configure SPI pins to bit-bang
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(cs, OUTPUT);
  pinMode(dc, OUTPUT);
  pinMode(rst, OUTPUT);

  // Begin transaction by pulling cs and dc LOW
  digitalWrite(cs, LOW);
  digitalWrite(dc, LOW);
  digitalWrite(SCK, LOW);
  digitalWrite(rst, HIGH);

  // Write to read register 0x71
  uint8_t cmd = 0x71;
  for (int i = 0; i < 8; i++) {
    digitalWrite(MOSI, (cmd & (1 << (7 - i))) != 0);
    digitalWrite(SCK, HIGH);
    digitalWrite(SCK, LOW);
  }

  // Set DC high to indicate data and switch MOSI to input with PUR in case
  // SSD1680 does not send data back
  digitalWrite(dc, HIGH);
  delayMicroseconds(1);
  pinMode(MOSI, INPUT_PULLUP);
  delayMicroseconds(1);

  // Read response from register
  uint8_t status = 0;
  for (int i = 0; i < 8; i++) {
    status <<= 1;
    if (digitalRead(MOSI)) {
      status |= 1;
    }
    digitalWrite(SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(SCK, LOW);
    delayMicroseconds(1);
  }

  // End transaction by pulling CS high
  digitalWrite(cs, HIGH);

  // Put back MOSI pin as an output
  pinMode(MOSI, OUTPUT);

  WS_DEBUG_PRINT("[display] Bitbang read 0x71: 0x");
  WS_DEBUG_PRINTLN(status, HEX);

  return status == 0xFF;
}