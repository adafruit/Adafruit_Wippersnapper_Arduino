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

  _neopixel = new Adafruit_NeoPixel((uint16_t)num_pixels, pin_data, order);
  _neopixel->begin();
  _neopixel->setBrightness((uint8_t)brightness);
  _neopixel->clear();
  _neopixel->show();
  // Check if the NeoPixel object was created successfully
  if (_neopixel->numPixels() != num_pixels)
    return false;

  WS_DEBUG_PRINT("[pixels] Added NeoPixel strand on pin ");
  WS_DEBUG_PRINT(pin_data);
  return true;
}

bool PixelsHardware::ConfigureStrand(wippersnapper_pixels_PixelsType type,
                                     wippersnapper_pixels_PixelsOrder order,
                                     uint32_t num_pixels, uint32_t brightness,
                                     const char *pin_data,
                                     const char *pin_clock) {
  _type = type;
  // Convert the pin string to an integer
  uint16_t p_data = atoi(pin_data + 1);
  // pin_clock is OPTIONALLY passed for a dotstar
  if (pin_clock != nullptr)
    uint16_t p_clock = atoi(pin_clock + 1);
  // Generics, TODO

  // TODO: Wrap the initialization into a function instead of within the
  // conditional
  if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    if (getStatusNeoPixelPin() == p_data && WsV2.lockStatusNeoPixelV2) {
      ReleaseStatusPixel(); // Release the status pixel for use
    }
    if (!AddNeoPixel(num_pixels, p_data, GetStrandOrder(order),
                     (uint8_t)brightness)) {
      WS_DEBUG_PRINTLN("[pixels] Failed to create NeoPixel strand!");
      return false;
    }
    return true;
  } else if (_type == wippersnapper_pixels_PixelsType_PIXELS_TYPE_DOTSTAR) {
    // TODO! DOTSTAR
  } else {
    // TODO! Signal!!!
    return false;
  }
  return true;
}

void PixelsHardware::begin() {
  // TODO:
  // https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/blob/main/src/components/pixels/ws_pixels.cpp#L258
}

neoPixelType
PixelsHardware::GetStrandOrder(wippersnapper_pixels_PixelsOrder order) {
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

/**************************************************************************/
/*!
    @brief  Sets the color of all pixels in the strand
    @param  pin_data
            Data pin for the pixel strand
    @param  color
            32-bit color value
*/
/**************************************************************************/
void PixelsHardware::SetPixelColor(uint8_t pin_data, uint32_t color) {}

/**************************************************************************/
/*!
    @brief  Deinitializes a pixel strand
    @param  pin_data
            Data pin for the pixel strand
*/
/**************************************************************************/
void PixelsHardware::deinit(uint8_t pin_data) {}