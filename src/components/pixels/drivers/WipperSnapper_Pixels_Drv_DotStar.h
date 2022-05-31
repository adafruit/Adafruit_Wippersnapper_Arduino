/*!
 * @file WipperSnapper_Pixels_Drv_DotStar.h
 *
 * Component driver for DotStar (APA102) Addressable RGB LEDs.
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
#ifndef WIPPERSNAPPER_PIXELS_DRV_DOTSTAR_H
#define WIPPERSNAPPER_PIXELS_DRV_DOTSTAR_H
#include "wippersnapper/pixels/v1/pixels.pb.h"
#include <Adafruit_DotStar.h>
#include <vector>

/**************************************************************************/
/*!
    @brief  Class that provides an interface w/addressable pixel
            components such as NeoPixel, DotStar.
*/
/**************************************************************************/
class WipperSnapper_Pixels_Drv_DotStar {
public:
  WipperSnapper_Pixels_Drv_DotStar();
  ~WipperSnapper_Pixels_Drv_DotStar();

  // DotStar Driver
  bool addDotStar(uint32_t pixelsNum, uint32_t pixelsBrightness,
                  wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig);
  bool updateDotStar(int8_t pixelBrightness,
                     wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig);
  bool deleteDotStar(wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig);
  bool fillDotStar(uint32_t pixelColor,
                   wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig);
  void deinitDotStars();
  bool isInitialized = false; ///< Has this class been initialized?

private:
  Adafruit_DotStar *_dotstar = nullptr; ///< Pointer to a dotstar object
  std::vector<Adafruit_DotStar *>
      _dotstars; ///< Vector of ptrs to dotstar objects
};

#endif // WIPPERSNAPPER_PIXELS_DRV_DOTSTAR_H