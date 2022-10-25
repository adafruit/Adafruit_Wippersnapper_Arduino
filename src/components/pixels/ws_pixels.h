/*!
 * @file ws_pixels.h
 *
 * High-level interface for wippersnapper to manage pixel strands
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * Brent Rubell for Adafruit Industries 2022
 *
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_PIXELS
#define WS_PIXELS

#include "Wippersnapper.h"

typedef struct strand_s {
  wippersnapper_pixels_v1_PixelsType
      pixels_type; ///< Strand type (NeoPixel, DotStar)
  // TODO: Unsure if we'll need pixels_ordering from init? Is this handled for
  // us by show()?
  int16_t pinNeoPixel;     ///< NeoPixel strand data pin
  int16_t pinDotStarData;  ///< DotStar strand data pin
  int16_t pinDotStarClock; ///< DotStar strand clock pin
} strand_t;

class Wippersnapper; ///< friend class
/**************************************************************************/
/*!
    @brief  Class for managing and interfacing with strands of addressable
            RGB LED pixels.
*/
/**************************************************************************/
class ws_pixels {
public:
  ws_pixels();
  ~ws_pixels();

  // TODO: Needs implementations, these interface with protobufs
  bool
  addStrand(wippersnapper_pixels_v1_PixelsCreateRequest *pixelsCreateReqMsg);
  void deleteStrand();
  void writeStrand();

private:
  // TODO
};
extern Wippersnapper WS;
#endif // WS_PIXELS