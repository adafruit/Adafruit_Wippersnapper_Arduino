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

// TODO: Possibly pass this the type so it can delete it??
// TODO: use this in destructor for all strandIdx
void ws_pixels::deallocateStrand(int16_t strandIdx) {
  // de-allocate the pin
  _strands[strandIdx] = {
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED,
      128,
      nullptr,
      nullptr,
      -1,
      -1,
      -1};
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

/**************************************************************************/
/*!
    @brief   Initializes a strand of addressable RGB Pixels.
    @param   pixelsCreateReqMsg
             Pointer to strand init. request message
    @returns Type of NeoPixel strand, usable by Adafruit_NeoPixel
             constructor
*/
/**************************************************************************/
bool ws_pixels::addStrand(
    wippersnapper_pixels_v1_PixelsCreateRequest *pixelsCreateReqMsg) {
  bool is_success = true;

  // attempt to allocate a free strand
  int16_t strandIdx = allocateStrand();
  if (strandIdx == -1)
    is_success = false;

  // unpack strand type
  _strands[strandIdx].type = pixelsCreateReqMsg->pixels_type;

  if (_strands[strandIdx].type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    char *pixelsPin = pixelsCreateReqMsg->pixels_pin_neopixel + 1;
    // is requested pin in-use by the status pixel?
    if (getStatusNeoPixelPin() == atoi(pixelsPin) && WS.lockStatusNeoPixel)
      releaseStatusLED(); // release it!
    _strands[strandIdx].pinNeoPixel =
        atoi(pixelsPin); // save into strand struct.
    _strands[strandIdx].brightness =
        DEFAULT_PIXEL_BRIGHTNESS; // save default brightness
    // Get type of strand
    neoPixelType strandType =
        getStrandType(pixelsCreateReqMsg->pixels_ordering);
    // Create a new strand of NeoPixels
    _strands[strandIdx].neoPixelPtr = new Adafruit_NeoPixel(
        pixelsCreateReqMsg->pixels_num, atoi(pixelsPin), strandType);
    // initialize NeoPixel
    _strands[strandIdx].neoPixelPtr->begin();
    _strands[strandIdx].neoPixelPtr->setBrightness(
        _strands[strandIdx].brightness);
    _strands[strandIdx].neoPixelPtr->clear();
    _strands[strandIdx].neoPixelPtr->show();
    WS_DEBUG_PRINT("Created NeoPixel strand of length ");
    WS_DEBUG_PRINT(pixelsCreateReqMsg->pixels_num);
    WS_DEBUG_PRINT(" on GPIO #");
    WS_DEBUG_PRINTLN(pixelsCreateReqMsg->pixels_pin_neopixel);
  } else if (pixelsCreateReqMsg->pixels_type ==
             wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    // TODO: fill out dotstar
  } else {
    // err: unable to find pixels_type
    is_success = false;
  }

  // fill `wippersnapper_pixels_v1_PixelsCreateResponse` message
  size_t msgSz; // message's encoded size
  wippersnapper_signal_v1_PixelsResponse msgInitResp =
      wippersnapper_signal_v1_PixelsResponse_init_zero;
  msgInitResp.which_payload =
      wippersnapper_signal_v1_PixelsResponse_resp_pixels_create_tag;
  msgInitResp.payload.resp_pixels_create.pixels_num =
      (uint32_t)_strands[strandIdx].neoPixelPtr->numPixels();
  msgInitResp.payload.resp_pixels_create.pixels_type =
      pixelsCreateReqMsg->pixels_type;

  // publish `wippersnapper_pixels_v1_PixelsCreateResponse` message back to
  // broker
  memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
  pb_ostream_t ostream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&ostream, wippersnapper_signal_v1_PixelsResponse_fields,
                 &msgInitResp)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode "
                     "wippersnapper_signal_v1_PixelsResponse message!");
    return false;
  }
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_PixelsResponse_fields,
                      &msgInitResp);
  WS_DEBUG_PRINT("-> wippersnapper_signal_v1_PixelsResponse");
  WS._mqtt->publish(WS._topic_signal_ds18_device, WS._buffer_outgoing, msgSz,
                    1);
  WS_DEBUG_PRINTLN("Published!");

  return true;
}

int ws_pixels::getStrandIdx(int16_t pin) {
    int strandIdx = -1;
    for (int i = 0; i < sizeof(_strands); i++) {
      if (_strands[i].pinNeoPixel == pin)
        strandIdx = i;
    }
    return strandIdx;
}

void ws_pixels::deleteStrand(
    wippersnapper_pixels_v1_PixelsDeleteRequest *pixelsDeleteMsg) {
  int pinData, strandIdx;

  pinData = atoi(pixelsDeleteMsg->pixels_pin_data + 1);
  // check addressable pixel strand type
  switch (pixelsDeleteMsg->pixels_type)
  {
  case wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL:
    strandIdx = getStrandIdx(pinData);
    if (strandIdx == -1) {
      WS_DEBUG_PRINTLN("ERROR: NeoPixel strand not found, can not delete strand!");
      return;
    }
    // de-init and release NeoPixel object
    delete _strands[strandIdx].neoPixelPtr;
    // if NeoPixel strand was the builtin status LED - re-init status LED behavior
    if (pinData == STATUS_NEOPIXEL_PIN)
      initStatusLED(); // re-use this pixel as a status LED again */
    break;
  default:
    break;
  }

  // deallocate strand object at strandIdx
  // TODO: Do we do this for Dotstar as well?
  deallocateStrand(strandIdx);
}

void ws_pixels::writeStrand(wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg) {
  // check type of strand we're writing to
  
}
