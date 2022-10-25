/*!
 * @file ws_pixels.cpp
 *
 * High-level interface for wippersnapper to manage pixel strands
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * Written by Brent Rubell for Adafruit Industries, 2022.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "ws_pixels.h"

/**************************************************************************/
/*!
    @brief  Constructor
*/
/**************************************************************************/
ws_pixels::ws_pixels() {
  // init array of strands using aggregate list
  for (int i = 0; i < sizeof(_strands); i++)
    _strands[i] = {wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED,
                   128,
                   nullptr,
                   nullptr,
                   -1,
                   -1,
                   -1};
}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_pixels::~ws_pixels() {
  // TODO!
  // deinitialize objects
  // release pins back
}

int16_t ws_pixels::allocateStrand() {
  int16_t strandIdx = 0;
  for (strandIdx; strandIdx < sizeof(_strands); strandIdx++) {
    if (_strands[strandIdx].type ==
        wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED)
      return strandIdx;
  }
  // unable to find a free strand
  return -1;
}

bool ws_pixels::addStrand(
    wippersnapper_pixels_v1_PixelsCreateRequest *pixelsCreateReqMsg) {
  bool is_success = true;

  // attempt to allocate a free strand
  int16_t strandIdx = allocateStrand();
  if (strandIdx == -1)
    is_success = false;

  // TODO: Maybe we could encapsulate this as well so we can control this flow
  // with is_success
  switch (pixelsCreateReqMsg->pixels_type) {
  case wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL:
    // unpack pin, TODO!
    // char *pixelsPin = pixelsCreateReqMsg->pixels_pin_neopixel + 1;
    // (uint16_t n, int16_t p, neoPixelType t
    // TODO: neoPixelType and pin is hard-coded, we need to remove this!
    _strands[strandIdx].neoPixelPtr =
        new Adafruit_NeoPixel(pixelsCreateReqMsg->pixels_num, 16, NEO_WRBG);
    // check if pin or num pixels were not set
    if (_strands[strandIdx].neoPixelPtr->getPin() == -1 &&
        _strands[strandIdx].neoPixelPtr->numPixels() == 0) {
      is_success = false;
      break;
    }
    // init. strip
    _strands[strandIdx].neoPixelPtr->begin();
    // set all pixel colors to off and send to strand
    _strands[strandIdx].neoPixelPtr->clear();
    _strands[strandIdx].neoPixelPtr->show();
    break;
  case wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR:
    /* code */
    break;
  default:
    break;
  }

  return false;
}

void ws_pixels::deleteStrand() {
  // TODO!
}

void ws_pixels::writeStrand() {
  // TODO
}
