/*!
 * @file WipperSnapper_Pixels_Drv_NeoPixel.h
 *
 * Component driver for NeoPixels
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_PIXELS_DRV_NEOPIXEL_H
#define WIPPERSNAPPER_PIXELS_DRV_NEOPIXEL_H
#include "wippersnapper/pixels/v1/pixels.pb.h"
#include <Adafruit_NeoPixel.h>
#include <vector>

/**************************************************************************/
/*!
    @brief  Class that provides an interface w/addressable pixel
            components such as NeoPixel, DotStar.
*/
/**************************************************************************/
class WipperSnapper_Pixels_Drv_NeoPixel {
public:
  WipperSnapper_Pixels_Drv_NeoPixel();
  ~WipperSnapper_Pixels_Drv_NeoPixel();

  bool addNeoPixel(uint32_t pixelsNum, uint32_t pixelsBrightness,
                   wippersnapper_pixels_v1_NeoPixelConfig neoPixelInitMsg);
  bool updateNeoPixel(int8_t pixelBrightness,
                      wippersnapper_pixels_v1_NeoPixelConfig msgNeoPixelConfig);
  bool deleteNeoPixel(wippersnapper_pixels_v1_NeoPixelConfig msgNeoPixelConfig);
  bool fillNeoPixel(uint32_t pixelColor,
                    wippersnapper_pixels_v1_NeoPixelConfig msgNeoPixelConfig);

private:
  Adafruit_NeoPixel *_neopixel = nullptr; ///< Pointer to a neopixel object
  std::vector<Adafruit_NeoPixel *>
      _neopixels; ///< Vector of ptrs to neopixel objects
};

#endif // WIPPERSNAPPER_PIXELS_DRV_NEOPIXEL_H