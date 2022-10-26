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

/******************************************************************************/
/*!
    @brief   Allocates an index of a free strand_t within the strand array.
    @returns Index of free strand_t, -1 if strand array is full.
*/
/******************************************************************************/
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

/**************************************************************************/
/*!
    @brief   Returns the `neoPixelType` provided the strand's pixelOrder
    @param   pixelOrder
             Desired pixel order, from init. message.
    @returns Type of NeoPixel strand, usable by Adafruit_NeoPixel
             constructor
*/
/**************************************************************************/
neoPixelType getStrandType(wippersnapper_pixels_v1_PixelsOrder pixelOrder) {
  neoPixelType strandType;
  switch (pixelOrder) {
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GRB:
    strandType = NEO_GRB + NEO_KHZ800;
    break;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GRBW:
    strandType = NEO_GRBW + NEO_KHZ800;
    break;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RGB:
    strandType = NEO_RGB + NEO_KHZ800;
    break;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RGBW:
    strandType = NEO_RGBW + NEO_KHZ800;
    break;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_BRG:
    strandType = NEO_BRG + NEO_KHZ800;
    break;
  default:
    break;
  }
  return strandType;
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

    // Get type of strand
    neoPixelType strandType =
        getStrandType(pixelsCreateReqMsg->pixels_ordering);
    // Create a new strand of NeoPixels
    _strands[strandIdx].neoPixelPtr = new Adafruit_NeoPixel(
        pixelsCreateReqMsg->pixels_num, atoi(pixelsPin), strandType);
    // check if pin or num pixels were not set by constructor
    if (_strands[strandIdx].neoPixelPtr->getPin() == -1 &&
        _strands[strandIdx].neoPixelPtr->numPixels() == 0)
      is_success = false;
    // TODO: Complete init routine
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
