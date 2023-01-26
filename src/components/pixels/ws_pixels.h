/*!
 * @file ws_pixels.h
 *
 * High-level interface for wippersnapper to manage addressable RGB pixel
 * strands
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * Brent Rubell for Adafruit Industries, 2022-2023
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

#define ERR_INVALID_STRAND -1 ///< Invalid strand index

/** Object representation of a strand of pixels */
struct strand_s {
  Adafruit_NeoPixel *neoPixelPtr; ///< Ptr to a NeoPixel object
  Adafruit_DotStar *dotStarPtr;   ///< Ptr to a DotStar object
  wippersnapper_pixels_v1_PixelsType
      type;           ///< Type of strand (DotStar, NeoPixel)
  uint8_t brightness; ///< Strand brightness (0 to 255)
  uint16_t numPixels; ///< Number of pixels on strand
  wippersnapper_pixels_v1_PixelsOrder ordering; ///< Color order of strand
  int16_t pinNeoPixel;                          ///< NeoPixel strand data pin
  int16_t pinDotStarData;                       ///< DotStar strand data pin
  int16_t pinDotStarClock;                      ///< DotStar strand clock pin
};

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
  void fillStrand(wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg);

  // Helpers
  int16_t allocateStrand();
  void deallocateStrand(int16_t strandIdx);
  int getStrandIdx(int16_t pin, wippersnapper_pixels_v1_PixelsType type);
  neoPixelType
  getNeoPixelStrandOrder(wippersnapper_pixels_v1_PixelsOrder pixelOrder);
  uint8_t getDotStarStrandOrder(wippersnapper_pixels_v1_PixelsOrder pixelOrder);
  void publishAddStrandResponse(bool is_success, char *pixels_pin_data);
};
extern Wippersnapper WS;
#endif // WS_PIXELS