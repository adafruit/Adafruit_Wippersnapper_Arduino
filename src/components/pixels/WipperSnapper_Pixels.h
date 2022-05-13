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
  addPixel(); // high-level, PixelsCreate
  updatePixel(); // high-level, PixelsUpdate
  deletePixel(); // high-level, PixelsDelete
  fillPixel(); // high-level, PixelsFillAll

  // NeoPixel Driver
  addNeoPixel();
  updateNeoPixel();
  deleteNeoPixel();
  fillNeoPixel();

  // DotStar Driver
  addDotStar();
  updateDotStar();
  deleteDotStar();
  fillDotStar();

private:
  // !!!TODO!!!
  // vector of pixel driver
  // vector of dotstar driver
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_PIXELS_H