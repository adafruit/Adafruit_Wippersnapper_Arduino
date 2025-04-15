/*!
 * @file hardware.cpp
 *
 * Hardware implementation for pixel strands.
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

/**************************************************************************/
/*!
    @brief  Constructs a new PixelsHardware object
*/
/**************************************************************************/
PixelsHardware::PixelsHardware() {
  _type = wippersnapper_pixels_PixelsType_PIXELS_TYPE_UNSPECIFIED;
}

/**************************************************************************/
/*!
    @brief  Destructs a PixelsHardware object
*/
/**************************************************************************/
PixelsHardware::~PixelsHardware() {}

bool PixelsHardware::AddNeoPixel(uint16_t num_pixels, uint16_t pin_data,
                                 neoPixelType order, uint8_t brightness) {
  if (getStatusNeoPixelPin() == pin_data && WsV2.lockStatusNeoPixelV2)
    ReleaseStatusPixel(); // Release the status pixel for use

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

bool PixelsHardware::AddDotStar(uint16_t num_pixels, uint16_t pin_data,
                                uint16_t pin_clock,
                                wippersnapper_pixels_PixelsOrder order,
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
}

bool PixelsHardware::AddStrand(wippersnapper_pixels_PixelsType type,
                               wippersnapper_pixels_PixelsOrder order,
                               uint32_t num_pixels, uint32_t brightness,
                               const char *pin_data, const char *pin_clock) {
  _type = type;
  // Convert the pin string to an integer
  uint16_t p_data = atoi(pin_data + 1);
  if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    if (!AddNeoPixel(num_pixels, p_data, GetStrandOrderNeoPixel(order),
                     (uint8_t)brightness)) {
      WS_DEBUG_PRINTLN("[pixels] Failed to create NeoPixel strand!");
      return false;
    }
    return true;
  } else if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_DOTSTAR) {
    uint16_t p_clock = atoi(pin_clock + 1);
    if (!AddDotStar(num_pixels, p_data, p_clock, order, (uint8_t)brightness)) {
      WS_DEBUG_PRINTLN("[pixels] Failed to create DotStar strand!");
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the color of all pixels in the strand
    @param  pin_data
            Data pin for the pixel strand
    @param  color
            32-bit color value
*/
/**************************************************************************/
void PixelsHardware::FillStrand(uint32_t color) {
  // Apply gamma correction to match IO Web
  uint32_t color_gamma = ApplyGammaCorrection(color);
  WS_DEBUG_PRINT("[pixels] Filling strand with color: ");
  WS_DEBUG_PRINT(color_gamma, HEX);
  if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    _neopixel->fill(color_gamma);
    _neopixel->show();
  } else if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_DOTSTAR) {
    _dotstar->fill(color_gamma);
    _dotstar->show();
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
  }
}

uint32_t PixelsHardware::ApplyGammaCorrection(uint32_t color) {
  if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    return _neopixel->gamma32(color);
  } else if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_DOTSTAR) {
    return _dotstar->gamma32(color);
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
    return color;
  }
}

/**************************************************************************/
/*!
    @brief  Deinitializes a pixel strand
    @param  pin_data
            Data pin for the pixel strand
*/
/**************************************************************************/
void PixelsHardware::RemoveStrand() {
  if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    if (_neopixel != nullptr) {
      delete _neopixel;
      _neopixel = nullptr;
    }
  } else if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_DOTSTAR) {
    if (_dotstar != nullptr) {
      delete _dotstar;
      _dotstar = nullptr;
    }
  } else {
    WS_DEBUG_PRINTLN("[pixels] Unknown pixel type!");
  }

  // Optionally re-init the status pixel for reuse by app.
  if (getStatusNeoPixelPin() == _pin_data && !WsV2.lockStatusNeoPixelV2)
    initStatusLED();
}

/**************************************************************************/
/**
 * @brief Gets the data pin used by the pixel strand
 *
 * @return The desired data pin
 */
/**************************************************************************/
uint16_t PixelsHardware::GetPinData() { return _pin_data; }

neoPixelType
PixelsHardware::GetStrandOrderNeoPixel(wippersnapper_pixels_PixelsOrder order) {
  switch (order) {
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_GRB:
    return NEO_GRB + NEO_KHZ800;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_GRBW:
    return NEO_GRBW + NEO_KHZ800;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_RGB:
    return NEO_RGB + NEO_KHZ800;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_RGBW:
    return NEO_RGBW + NEO_KHZ800;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_BRG:
    return NEO_BRG + NEO_KHZ800;
  default:
    return NEO_GRB + NEO_KHZ800;
  }
}

uint8_t
PixelsHardware::GetStrandOrderDotStar(wippersnapper_pixels_PixelsOrder order) {
  switch (order) {
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_GRB:
    return DOTSTAR_GRB;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_RGB:
    return DOTSTAR_RGB;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_BRG:
    return DOTSTAR_BRG;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_GBR:
    return DOTSTAR_GBR;
  case wippersnapper_pixels_PixelsOrder_PIXELS_ORDER_BGR:
    return DOTSTAR_BGR;
  default:
    return DOTSTAR_BRG;
  }
}
