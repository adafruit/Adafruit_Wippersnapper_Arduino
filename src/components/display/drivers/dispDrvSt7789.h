/*!
 * @file src/components/display/drivers/dispDrvSt7789.h
 *
 * Driver for ST7789-based TFT displays.
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
#ifndef WS_DISP_DRV_ST7789
#define WS_DISP_DRV_ST7789

#include "dispDrvBase.h"
#include <Adafruit_ST7789.h>

#define ST7789_TEXT_SZ_DEFAULT 2 ///< Default text size for ST7789 displays

/*!
    @brief  Driver for ST7789-based TFT displays.
*/
class dispDrvSt7789 : public dispDrvBase {
public:
  /*!
      @brief  Constructor for the ST7789 display driver.
      @param  cs
              Chip Select pin for the display.
      @param  dc
              Data/Command pin for the display.
      @param  mosi
              MOSI pin for the display.
      @param  sck
              SCK pin for the display.
      @param  rst
              Optional Reset pin for the display.
      @param  miso
              Optional MISO pin for the display.
  */
  dispDrvSt7789(int16_t cs, int16_t dc, int16_t mosi, int16_t sck,
                int16_t rst = -1, int16_t miso = -1)
      : dispDrvBase(cs, dc, mosi, sck, rst, miso), _display(nullptr) {}

  ~dispDrvSt7789() {
    if (_display) {
      delete _display;
      _display = nullptr;
    }
  }

  /*!
      @brief  Attempts to initialize the ST7789 TFT driver.
      @return True if the display was initialized successfully, false otherwise.
  */
  bool begin() override {

// Special power control configuration for
// boards with built-in TFTs 
#if defined(TFT_BACKLITE)
    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    #if defined(TFT_I2C_POWER)
    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);
    #endif // TFT_I2C_POWER
#endif

    _display = new Adafruit_ST7789(_pin_cs, _pin_dc, _pin_rst);
    if (!_display)
      return false;

    _display->init(_width, _height);
    _display->setRotation(_rotation);
    setTextSize(ST7789_TEXT_SZ_DEFAULT);
    _display->fillScreen(ST77XX_BLACK);
    _display->setTextColor(ST77XX_WHITE);
    return true;
  }

  /*!
      @brief  Sets the text size for the display.
      @param  s
              The text size to set.
      @note   This method overrides the base class method to provide specific
              functionality for the ST7789 driver.
  */
  void setTextSize(uint8_t s) override {
    if (!_display)
      return;
    _text_sz = s;
    _display->setTextSize(s);
  }

  /*!
      @brief  Writes a message to the display.
      @param  message
              The message to write to the display.
      @note   This method overrides the base class method to provide specific
              functionality for the ST7789 driver.
  */
  virtual void writeMessage(const char *message) override {
    if (_display == nullptr)
      return;

    // Start with a fresh display buffer
    _display->fillScreen(ST77XX_BLACK);
    int16_t y_idx = 0;
    _display->setCursor(0, y_idx);

    // Calculate the line height based on the text size (NOTE: base height is
    // 8px)
    int16_t line_height = 8 * _text_sz;
    uint16_t c_idx = 0;
    size_t msg_size = strlen(message);
    for (size_t i = 0; i < msg_size && c_idx < msg_size; i++) {
      if (y_idx + line_height > _height)
        break;
      if (message[i] == '\\' && i + 1 < msg_size &&
          (message[i + 1] == 'n' || message[i + 1] == 'r')) {
        // Handle \r\n sequence as a single newline
        if (message[i + 1] == 'r' && i + 3 < msg_size &&
            message[i + 2] == '\\' && message[i + 3] == 'n') {
          // Skip to the next line
          if (y_idx + line_height > _height)
            break;
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i += 3;
        } else if (message[i + 1] == 'n') {
          // Skip to the next line
          if (y_idx + line_height > _height)
            break;
          y_idx += line_height;
          _display->setCursor(0, y_idx);
          i++;
        }
      } else if (message[i] == 0xC2 && message[i + 1] == 0xB0) {
        _display->write(char(248));
        i++;
      } else {
        _display->print(message[i]);
      }
    }
  }

private:
  Adafruit_ST7789 *_display;
};

#endif // WS_DISP_DRV_ST7789