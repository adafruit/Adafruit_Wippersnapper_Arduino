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
#include "drivers/WipperSnapper_Pixels_Drv_DotStar.h"
#include "drivers/WipperSnapper_Pixels_Drv_NeoPixel.h"

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
  bool addPixel(wippersnapper_pixels_v1_PixelsCreate
                    msgPixelsCreate); // high-level, PixelsCreate
  bool updatePixel(wippersnapper_pixels_v1_PixelsUpdate
                       msgPixelsUpdate); // high-level, PixelsUpdate
  bool deletePixel(wippersnapper_pixels_v1_PixelsDelete
                       msgPixelsDelete); // high-level, PixelsDelete
  bool fillPixel(wippersnapper_pixels_v1_PixelsFillAll
                     msgPixelsFillAll); // high-level, PixelsFillAll
  void deInitPixel();

private:
  WipperSnapper_Pixels_Drv_NeoPixel _neoDriver;
  WipperSnapper_Pixels_Drv_DotStar _dotStarDriver;
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_PIXELS_H