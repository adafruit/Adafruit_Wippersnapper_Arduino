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
    uint8_t pin_data;           ///< Data pin
    uint8_t pin_clock;          ///< Clock pin (for DotStar)
    wippersnapper_pixels_PixelsType type;     ///< Pixel type
    wippersnapper_pixels_PixelsOrder order;   ///< Color ordering
    uint32_t num_pixels;        ///< Number of pixels
    uint32_t brightness;        ///< Current brightness (0-255)
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
  bool ConfigurePixelStrand(uint8_t pin_data, uint8_t pin_clock, 
                          wippersnapper_pixels_PixelsType type,
                          wippersnapper_pixels_PixelsOrder order,
                          uint32_t num_pixels, uint32_t brightness);
  void SetPixelColor(uint8_t pin_data, uint32_t color);
  void deinit(uint8_t pin_data);
private:
};
#endif // WS_PIXELS_HARDWARE_H