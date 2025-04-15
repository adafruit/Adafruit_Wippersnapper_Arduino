/*!
 * @file hardware.h
 *
 * Hardware interface for pixel strands.
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

// TODO: Do we need this?
/**
 * @struct PixelStrand
 * @brief This struct represents a NeoPixel or DotStar strand.
 */
struct PixelStrand {
  uint8_t pin_data;                       ///< Data pin
  uint8_t pin_clock;                      ///< Clock pin (for DotStar)
  wippersnapper_pixels_PixelsType type;   ///< Pixel type
  wippersnapper_pixels_PixelsOrder order; ///< Color ordering
  uint32_t num_pixels;                    ///< Number of pixels
  uint32_t brightness;                    ///< Current brightness (0-255)
};

/**************************************************************************/
/*!
    @brief  Interface for interacting with hardware's pixel strands.
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
  Adafruit_NeoPixel *_neopixel = nullptr; ///< Used for NeoPixel pixel strands
  Adafruit_DotStar *_dotstar = nullptr;   ///< Used for DotStar pixel strands
  wippersnapper_pixels_PixelsType _type;
  uint16_t _pin_data; ///< Data pin for the pixel strand
  bool AddNeoPixel(uint16_t num_pixels, uint16_t pin_data, neoPixelType order,
                   uint8_t brightness);
  bool AddDotStar(uint16_t num_pixels, uint16_t pin_data, uint16_t pin_clock,
                  wippersnapper_pixels_PixelsOrder order, uint8_t brightness);
  neoPixelType GetStrandOrderNeoPixel(wippersnapper_pixels_PixelsOrder order);
  uint8_t GetStrandOrderDotStar(wippersnapper_pixels_PixelsOrder order);
  uint32_t ApplyGammaCorrection(uint32_t color);
};
#endif // WS_PIXELS_HARDWARE_H