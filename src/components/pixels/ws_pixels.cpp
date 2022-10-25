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

  if (pixelsCreateReqMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    char *pixelsPin = pixelsCreateReqMsg->pixels_pin_neopixel + 1;
    // TODO: check if pin and object is being used by the StatusLED + deallocate
    // DO THIS ON TUESDAY
    // (uint16_t n, int16_t p, neoPixelType t
    // TODO: neoPixelType and pin is hard-coded, we need to remove this!
    _strands[strandIdx].neoPixelPtr =
        new Adafruit_NeoPixel(pixelsCreateReqMsg->pixels_num, 16, NEO_WRBG);
    // check if pin or num pixels were not set
    if (_strands[strandIdx].neoPixelPtr->getPin() == -1 &&
        _strands[strandIdx].neoPixelPtr->numPixels() == 0)
      is_success = false;
  } else if (pixelsCreateReqMsg->pixels_type ==
             wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    // TODO: fill out dotstar
  } else {
    // err: unable to find pixels_type
    is_success = false;
  }
  // TODO publish `_wippersnapper_pixels_v1_PixelsCreateResponse` back to broker

  return false;
}

void ws_pixels::deleteStrand() {
  // TODO!
}

void ws_pixels::writeStrand() {
  // TODO
}
