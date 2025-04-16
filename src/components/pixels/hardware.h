/*!
 * @file hardware.h
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
#ifndef WS_PIXELS_HARDWARE_H
#define WS_PIXELS_HARDWARE_H
#include "Wippersnapper_V2.h"

/**************************************************************************/
/*!
    @brief  Interface for interacting with NeoPixel or Dotstar
            pixel strands
*/
/**************************************************************************/
class PixelsHardware {
public:
  PixelsHardware();
  ~PixelsHardware();
  bool AddStrand(wippersnapper_pixels_PixelsType type,
                 wippersnapper_pixels_PixelsOrder order, uint32_t num_pixels,
                 uint32_t brightness, const char *pin_data,
                 const char *pin_clock);
  uint16_t GetPinData();
  void FillStrand(uint32_t color);
  void RemoveStrand();
private:
  Adafruit_NeoPixel *_neopixel = nullptr; ///< Used for NeoPixel strands
  Adafruit_DotStar *_dotstar = nullptr;   ///< Used for DotStar strands
  wippersnapper_pixels_PixelsType _type; ///< Holds the type of strand
  uint16_t _pin_data; ///< Data pin for the strand
  bool AddNeoPixel(uint16_t num_pixels, uint16_t pin_data, neoPixelType order,
                   uint8_t brightness);
  bool AddDotStar(uint16_t num_pixels, uint16_t pin_data, uint16_t pin_clock,
                  wippersnapper_pixels_PixelsOrder order, uint8_t brightness);
  neoPixelType GetStrandOrderNeoPixel(wippersnapper_pixels_PixelsOrder order);
  uint8_t GetStrandOrderDotStar(wippersnapper_pixels_PixelsOrder order);
  uint32_t ApplyGammaCorrection(uint32_t color);
};
#endif // WS_PIXELS_HARDWARE_H