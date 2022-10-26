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

#define MAX_PIXEL_STRANDS                                                      \
  5 ///< Maximum number of pixel strands connected to a WipperSnapper device

typedef struct strand_s {
  wippersnapper_pixels_v1_PixelsType type; ///< Strand type (NeoPixel, DotStar)
  uint32_t brightness;                     ///< Strand brightness (0 to 255)
  Adafruit_NeoPixel *neoPixelPtr;          ///< Ptr to a NeoPixel object
  Adafruit_DotStar *dotStarPtr;            ///< Ptr to a DotStar object
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

  int16_t allocateStrand();

private:
  strand_t _strands[MAX_PIXEL_STRANDS];
};
extern Wippersnapper WS;
#endif // WS_PIXELS