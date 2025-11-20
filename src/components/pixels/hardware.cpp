/*!
 * @file src/components/pixels/hardware.cpp
 *
 * Hardware interface for NeoPixel/DotStar strands.
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
#include "hardware.h"
#include "Wippersnapper_V2.h"

/*!
    @brief  Constructs a new PixelsHardware object
*/
PixelsHardware::PixelsHardware() { _type = ws_pixels_Type_T_UNSPECIFIED; }

/*!
    @brief  Destructs a PixelsHardware object
*/
PixelsHardware::~PixelsHardware() {}

/*!
    @brief  Initializes a new NeoPixel strand on a desired pin.
    @param  num_pixels
            Number of pixels in the strand.
    @param  pin_data
            The desired data pin for the pixel strand.
    @param  order
            The desired color ordering for the pixel strand.
    @param  brightness
            The desired brightness of the pixel strand (0-255).
    @returns True if successful, False otherwise.
*/
bool PixelsHardware::AddNeoPixel(uint16_t num_pixels, uint16_t pin_data,
                                 neoPixelType order, uint8_t brightness) {
  if (getStatusNeoPixelPin() == pin_data && WsV2.lockStatusNeoPixelV2) {
    WS_DEBUG_PRINTLN("[pixels] Releasing status pixel for use");
    ReleaseStatusPixel(); // Release the status pixel for use
  }

  _neopixel = new Adafruit_NeoPixel(num_pixels, pin_data, order);
  _neopixel->begin();
  _neopixel->setBrightness(brightness);
  _neopixel->clear();
  _neopixel->show();
  // Check if the NeoPixel object was created successfully
  if (_neopixel->numPixels() != num_pixels)
    return false;

  WS_DEBUG_PRINT("[pixels] Added NeoPixel strand on pin ");
  WS_DEBUG_PRINT(pin_data);
  return true;
}

/*!
    @brief  Initializes a new DotStar strand on a desired pin.
    @param  num_pixels
            Number of pixels in the strand.
    @param  pin_data
            The desired data pin for the pixel strand.
    @param  pin_clock
            The desired clock pin for the pixel strand.
    @param  order
            The desired color ordering for the pixel strand.
    @param  brightness
            The desired brightness of the pixel strand (0-255).
    @returns True if successful, False otherwise.
*/
bool PixelsHardware::AddDotStar(uint16_t num_pixels, uint16_t pin_data,
                                uint16_t pin_clock, ws_pixels_Order order,
                                uint8_t brightness) {
  if (getStatusDotStarDataPin() == pin_data && WsV2.lockStatusDotStarV2)
    ReleaseStatusPixel(); // Release the status pixel for use

  _dotstar = new Adafruit_DotStar(num_pixels, pin_data, pin_clock,
                                  GetStrandOrderDotStar(order));
  _dotstar->begin();
  _dotstar->setBrightness(brightness);
  _dotstar->clear();
  _dotstar->show();
  // Check if the DotStar object was created successfully
  if (_dotstar->numPixels() != num_pixels) {
    WS_DEBUG_PRINTLN("[pixels] Failed to create DotStar strand!");
    return false;
  }

  WS_DEBUG_PRINT("[pixels] Added DotStar strand on pin ");
  WS_DEBUG_PRINT(pin_data);
  WS_DEBUG_PRINT(" and clock pin ");
  WS_DEBUG_PRINT(pin_clock);
  return true;
}

/*!
    @brief  Initializes a new pixel strand on a desired pin.
    @param  type
            The desired pixel type (NeoPixel or DotStar).
    @param  order
            The desired color ordering for the pixel strand.
    @param  num_pixels
            Number of pixels in the strand.
    @param  brightness
            The desired brightness of the pixel strand (0-255).
    @param  pin_data
            The desired data pin for the pixel strand.
    @param  pin_clock
            The desired clock pin for the pixel strand (for DotStar).
    @returns True if successful, False otherwise.
*/
bool PixelsHardware::AddStrand(ws_pixels_Type type, ws_pixels_Order order,
                               uint32_t num_pixels, uint32_t brightness,
                               const char *pin_data, const char *pin_clock) {
  _type = type;
  // Convert the pin string to an integer
  uint16_t p_data = atoi(pin_data + 1);
  _pin_data = p_data; // Store the pin data for later reference

  if (_type == ws_pixels_Type_T_NEOPIXEL) {
    if (!AddNeoPixel(num_pixels, p_data, GetStrandOrderNeoPixel(order),
                     (uint8_t)brightness)) {
      WS_DEBUG_PRINTLN("[pixels] Failed to create NeoPixel strand!");
      return false;
    }
    return true;
  } else if (_type == ws_pixels_Type_T_DOTSTAR) {
    uint16_t p_clock = atoi(pin_clock + 1);
    if (!AddDotStar(num_pixels, p_data, p_clock, order, (uint8_t)brightness)) {
      WS_DEBUG_PRINTLN("[pixels] Failed to create DotStar strand!");
      return false;
    }
    return true;
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
    return false;
  }
  return true;
}

/*!
    @brief  Sets the color of all pixels in the strand
    @param  color
            32-bit color value
*/
void PixelsHardware::FillStrand(uint32_t color) {
  // Apply gamma correction to match IO Web
  uint32_t color_gamma = ApplyGammaCorrection(color);
  if (_type == ws_pixels_Type_T_NEOPIXEL) {
    _neopixel->fill(color_gamma);
    _neopixel->show();
  } else if (_type == ws_pixels_Type_T_DOTSTAR) {
    _dotstar->fill(color_gamma);
    _dotstar->show();
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
  }
}

/*!
    @brief  Applies gamma correction to a color value to match IO Web.
    @param  color
            The color value to be corrected
    @returns The gamma-corrected color value
*/
uint32_t PixelsHardware::ApplyGammaCorrection(uint32_t color) {
  if (_type == ws_pixels_Type_T_NEOPIXEL) {
    return _neopixel->gamma32(color);
  } else if (_type == ws_pixels_Type_T_DOTSTAR) {
    return _dotstar->gamma32(color);
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
    return color;
  }
}

/*!
    @brief  Deinitializes a pixel strand
*/
void PixelsHardware::RemoveStrand() {
  if (_type == ws_pixels_Type_T_NEOPIXEL) {
    if (_neopixel != nullptr) {
      _neopixel->clear();
      _neopixel->show();
      delete _neopixel;
      _neopixel = nullptr;
    }
  } else if (_type == ws_pixels_Type_T_DOTSTAR) {
    if (_dotstar != nullptr) {
      _dotstar->clear();
      _dotstar->show();
      delete _dotstar;
      _dotstar = nullptr;
    }
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
  }
  // Optionally re-init the status pixel for reuse by app.
  if (getStatusNeoPixelPin() == _pin_data && !WsV2.lockStatusNeoPixelV2) {
    WS_DEBUG_PRINTLN("[pixels] Re-initializing status pixel");
    initStatusLED();
  }
  WS_DEBUG_PRINT("[pixels] Removed pixel strand from pin: ");
  WS_DEBUG_PRINTLN(_pin_data);

  _pin_data = 0; // Reset the pin data
  _type = ws_pixels_Type_T_UNSPECIFIED;
}

/**************************************************************************/
/**
 * @brief Gets the data pin used by the pixel strand
 *
 * @return The desired data pin
 */
/**************************************************************************/
uint16_t PixelsHardware::GetPinData() { return _pin_data; }

/*!
    @brief  Gets the color ordering for NeoPixel strands
    @param  order
            The desired color ordering for the pixel strand.
    @returns The color ordering for NeoPixel strands
*/
neoPixelType PixelsHardware::GetStrandOrderNeoPixel(ws_pixels_Order order) {
  switch (order) {
  case ws_pixels_Order_O_GRB:
    return NEO_GRB + NEO_KHZ800;
  case ws_pixels_Order_O_GRBW:
    return NEO_GRBW + NEO_KHZ800;
  case ws_pixels_Order_O_RGB:
    return NEO_RGB + NEO_KHZ800;
  case ws_pixels_Order_O_RGBW:
    return NEO_RGBW + NEO_KHZ800;
  case ws_pixels_Order_O_BRG:
    return NEO_BRG + NEO_KHZ800;
  default:
    return NEO_GRB + NEO_KHZ800;
  }
}

/*!
    @brief  Gets the color ordering for DotStar strands
    @param  order
            The desired color ordering for the pixel strand.
    @returns The color ordering for DotStar strands
*/
uint8_t PixelsHardware::GetStrandOrderDotStar(ws_pixels_Order order) {
  switch (order) {
  case ws_pixels_Order_O_GRB:
    return DOTSTAR_GRB;
  case ws_pixels_Order_O_RGB:
    return DOTSTAR_RGB;
  case ws_pixels_Order_O_BRG:
    return DOTSTAR_BRG;
  case ws_pixels_Order_O_GBR:
    return DOTSTAR_GBR;
  case ws_pixels_Order_O_BGR:
    return DOTSTAR_BGR;
  default:
    return DOTSTAR_BRG;
  }
}
