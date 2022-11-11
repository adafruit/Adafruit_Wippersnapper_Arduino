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
  uint8_t brightness;                      ///< Strand brightness (0 to 255)
  wippersnapper_pixels_v1_PixelsOrder ordering; ///< Strand pixel ordering
  Adafruit_NeoPixel *neoPixelPtr;          ///< Ptr to a NeoPixel object
  Adafruit_DotStar *dotStarPtr;            ///< Ptr to a DotStar object
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

  // Protobuf RPC
  bool
  addStrand(wippersnapper_pixels_v1_PixelsCreateRequest *pixelsCreateReqMsg);
  void
  deleteStrand(wippersnapper_pixels_v1_PixelsDeleteRequest *pixelsDeleteMsg);
  void writeStrand(wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg);
  // TODO: setStrandBrightness
  // void setStrandBrightness(wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg);

  // Helpers
  int16_t allocateStrand();
  void deallocateStrand(int16_t strandIdx);
  int getStrandIdx(int16_t pin, wippersnapper_pixels_v1_PixelsType type);

private:
  strand_t _strands[MAX_PIXEL_STRANDS]; ///< Array of `strand_t`.
};
extern Wippersnapper WS;
#endif // WS_PIXELS