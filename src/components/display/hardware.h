/*!
 * @file src/components/display/hardware.h
 *
 * Hardware interface for display components.
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
#ifndef WS_DISPLAY_HARDWARE_H
#define WS_DISPLAY_HARDWARE_H
#include "Wippersnapper.h"
#include "drivers/dispDrvBase.h"
#include "drivers/dispDrvSsd1306.h"
#include "drivers/dispDrvSt7789.h"
#include "drivers/dispDrvThinkInkGrayscale4Eaamfgn.h"
#include "drivers/dispDrvThinkInkGrayscale4T5.h"
#include <functional>
#include <map>

/*!
    @brief  Interface for interacting with display hardware (TFT, eInk,
            OLED, etc.)
            This class provides methods to initialize, write to, and
            manage the state of display hardware.
*/
class DisplayHardware {
public:
  DisplayHardware(const char *name);
  ~DisplayHardware();

  //
  // API for configuring the display hardware //
  //
  const char *getName();
  void setType(wippersnapper_display_v1_DisplayType type);
  wippersnapper_display_v1_DisplayType getType();
  bool beginEPD(wippersnapper_display_v1_EPDConfig *config,
                wippersnapper_display_v1_SpiConfig *spi_config);
  bool beginTft(wippersnapper_display_v1_TftConfig *config,
                wippersnapper_display_v1_SpiConfig *spi_config);
  bool beginOled(wippersnapper_display_v1_SSD1306Config *config,
                 wippersnapper_display_v1_I2cConfig *i2c_config);

  //
  // API for Adafruit_GFX that abstracts hardware functionality
  // NOTE: These methods are meant to be implemented within dispDrvBase and
  // exposed within dispDrv driver instances
  //
  void writeMessage(const char *message);

private:
  int16_t parsePin(const char *pinStr);
  void removeSuffix(const char *suffix);
  bool detect_ssd1680(uint8_t cs, uint8_t dc, uint8_t rst);
  char _name[64]; ///< Identifies the hardware instance
  wippersnapper_display_v1_DisplayType _type; ///< Display type
  dispDrvBase *_drvDisp = nullptr;            ///< Base display driver
};
#endif // WS_DISPLAY_HARDWARE_H