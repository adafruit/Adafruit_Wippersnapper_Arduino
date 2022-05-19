/*!
 * @file WipperSnapper_Pixels.h
 *
 * This component interfaces with addressable pixel components.
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
#ifndef WIPPERSNAPPER_PIXELS_H
#define WIPPERSNAPPER_PIXELS_H
#include "Wippersnapper.h"
#include <Adafruit_DotStar.h>
#include <Adafruit_NeoPixel.h>

/**************************************************************************/
/*!
    @brief  Class that provides an interface w/addressable pixel
            components such as NeoPixel, DotStar.
*/
/**************************************************************************/
class WipperSnapper_Pixels {
public:
  WipperSnapper_Pixels();
  ~WipperSnapper_Pixels();
  void addPixel(wippersnapper_pixels_v1_PixelsCreate
                    msgPixelsCreate); // high-level, PixelsCreate
  void updatePixel(wippersnapper_pixels_v1_PixelsUpdate
                       msgPixelsUpdate); // high-level, PixelsUpdate
  void deletePixel(wippersnapper_pixels_v1_PixelsDelete
                       msgPixelsDelete); // high-level, PixelsDelete
  void fillPixel(wippersnapper_pixels_v1_PixelsFillAll
                     msgPixelsFillAll); // high-level, PixelsFillAll

  // NeoPixel Driver
  void addNeoPixel(uint32_t pixelsNum, uint32_t pixelsBrightness,
                   wippersnapper_pixels_v1_NeoPixelInit neoPixelInitMsg);
  void updateNeoPixel(int8_t pixelBrightness,
                      wippersnapper_pixels_v1_NeoPixelInit msgNeoPixelConfig);
  void deleteNeoPixel(wippersnapper_pixels_v1_NeoPixelInit msgNeoPixelConfig);
  void fillNeoPixel();

  // DotStar Driver
  void addDotStar();
  void updateDotStar();
  void deleteDotStar();
  void fillDotStar();

private:
  Adafruit_NeoPixel *_neopixel;                ///< Pointer to a neopixel object
  std::vector<Adafruit_NeoPixel *> _neopixels; ///< List of neopixels
  // std::vector<Adafruit_DotStar> dotstars; ///< List of neopixels
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_PIXELS_H