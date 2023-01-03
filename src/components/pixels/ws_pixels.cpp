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

strand_s _strands[MAX_PIXEL_STRANDS];

ws_pixels::ws_pixels() {
  
}
/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_pixels::~ws_pixels() {
  // de-allocate all strands
  for (int i = 0; i < sizeof(_strands); i++)
    deallocateStrand(i);
}

/**************************************************************************/
/*!
    @brief  Deallocates a `strand_t` within `_strands`, provided an index.
    @param  strandIdx
            The desired index of a `strand_t` within `_strands`.
*/
/**************************************************************************/
void ws_pixels::deallocateStrand(int16_t strandIdx) {
  // reset pixel type
  _strands[strandIdx].type =
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED;

  // delete the pixel type object
  if (_strands[strandIdx].neoPixelPtr != nullptr)
    delete _strands[strandIdx].neoPixelPtr;
  if ((_strands[strandIdx].dotStarPtr != nullptr))
    delete _strands[strandIdx].dotStarPtr;

  // was this pixel previously used as a status LED?
  if (_strands[strandIdx].pinNeoPixel == getStatusNeoPixelPin() ||
      _strands[strandIdx].pinDotStarData == getStatusDotStarDataPin())
    initStatusLED();
}

/******************************************************************************/
/*!
    @brief   Allocates an index of a free strand_t within the strand array.
    @returns Index of a free strand_t, -1 if strand array is full.
*/
/******************************************************************************/
int16_t ws_pixels::allocateStrand() {
  strand_s strands[5];
  WS_DEBUG_PRINTLN(strands[0].pinNeoPixel);
  _strands[0].brightness = 0;
  WS_DEBUG_PRINT("_strands[0].type: ");
  WS_DEBUG_PRINTLN(_strands[0].brightness);
  for (int16_t strandIdx = 0; strandIdx < MAX_PIXEL_STRANDS; strandIdx++) {
    if (_strands[strandIdx].type == wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED) {
      return strandIdx;
    }
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
neoPixelType
getNeoPixelStrandType(wippersnapper_pixels_v1_PixelsOrder pixelOrder) {
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
    strandType = NEO_GRB + NEO_KHZ800;
    break;
  }
  return strandType;
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
uint8_t getDotStarStrandOrder(wippersnapper_pixels_v1_PixelsOrder pixelOrder) {
  uint8_t order;
  if (pixelOrder == wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GRB) {
    order = DOTSTAR_GRB;
  } else if (pixelOrder ==
             wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RGB) {
    order = DOTSTAR_RGB;
  } else if (pixelOrder ==
             wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_BRG) {
    order = DOTSTAR_BRG;
  } else {
    order = -1;
  }
  return order;
}

/**************************************************************************/
/*!
    @brief   Initializes a strand of addressable RGB Pixels.
    @param   pixelsCreateReqMsg
             Pointer to strand init. request message
    @returns True if successfully initialized, False otherwise.
*/
/**************************************************************************/
bool ws_pixels::addStrand(
    wippersnapper_pixels_v1_PixelsCreateRequest *pixelsCreateReqMsg) {
  bool is_success = true;

  // attempt to allocate a free strand
  int16_t strandIdx = allocateStrand();
  WS_DEBUG_PRINT("Strand Idx: ");
  WS_DEBUG_PRINTLN(strandIdx);
  if (strandIdx == -1)
    is_success = false;

  // unpack strand type
  WS_DEBUG_PRINT("pixelsCreateReqMsg->pixels_type: ");
  WS_DEBUG_PRINTLN(pixelsCreateReqMsg->pixels_type);

  _strands[strandIdx].brightness = pixelsCreateReqMsg->pixels_brightness;
  _strands[strandIdx].type = pixelsCreateReqMsg->pixels_type;

  if (_strands[strandIdx].type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    char *pixelsPin = pixelsCreateReqMsg->pixels_pin_neopixel + 1;
    // is requested pin in-use by the status pixel?
    if (getStatusNeoPixelPin() == atoi(pixelsPin) && WS.lockStatusNeoPixel)
      releaseStatusLED(); // release it!
    _strands[strandIdx].pinNeoPixel =
        atoi(pixelsPin); // save into strand struct.
    _strands[strandIdx].brightness = pixelsCreateReqMsg->pixels_brightness;
    _strands[strandIdx].ordering = pixelsCreateReqMsg->pixels_ordering;
    // Get type of strand
    neoPixelType strandType =
        getNeoPixelStrandType(pixelsCreateReqMsg->pixels_ordering);
    // Create a new strand of NeoPixels
    _strands[strandIdx].neoPixelPtr = new Adafruit_NeoPixel(
        pixelsCreateReqMsg->pixels_num, atoi(pixelsPin), strandType);
    // initialize NeoPixel
    _strands[strandIdx].neoPixelPtr->begin();
    _strands[strandIdx].neoPixelPtr->setBrightness(
        _strands[strandIdx].brightness);
    _strands[strandIdx].neoPixelPtr->clear();
    _strands[strandIdx].neoPixelPtr->show();
    // post-init check
    if (_strands[strandIdx].neoPixelPtr->numPixels() == 0)
      is_success = false;
    WS_DEBUG_PRINT("Created NeoPixel strand of length ");
    WS_DEBUG_PRINT(pixelsCreateReqMsg->pixels_num);
    WS_DEBUG_PRINT(" on GPIO #");
    WS_DEBUG_PRINTLN(pixelsCreateReqMsg->pixels_pin_neopixel);
  }

  if (pixelsCreateReqMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    // unpack pins
    char *pinData = pixelsCreateReqMsg->pixels_pin_dotstar_data + 1;
    char *pinClock = pixelsCreateReqMsg->pixels_pin_dotstar_clock + 1;
    // fill strand_t with fields from `pixelsCreateReqMsg`
    _strands[strandIdx].type = pixelsCreateReqMsg->pixels_type;
    _strands[strandIdx].brightness = pixelsCreateReqMsg->pixels_brightness;
    _strands[strandIdx].ordering = pixelsCreateReqMsg->pixels_ordering;
    _strands[strandIdx].pinDotStarData = atoi(pinData);
    _strands[strandIdx].pinDotStarClock = atoi(pinClock);
    // release the status dotstar, if it is both in-use and the pin within
    // `pixelsCreateReqMsg`
    if ((_strands[strandIdx].pinDotStarData == getStatusDotStarDataPin()) &&
        WS.lockStatusDotStar)
      releaseStatusLED();
    // init DotStar object
    _strands[strandIdx].dotStarPtr = new Adafruit_DotStar(
        pixelsCreateReqMsg->pixels_num, _strands[strandIdx].pinDotStarData,
        _strands[strandIdx].pinDotStarClock,
        getDotStarStrandOrder(pixelsCreateReqMsg->pixels_ordering));
    // init. strand for output
    _strands[strandIdx].dotStarPtr->begin();
    _strands[strandIdx].dotStarPtr->setBrightness(
        _strands[strandIdx].brightness);
    _strands[strandIdx].dotStarPtr->clear();
    _strands[strandIdx].dotStarPtr->show();

    // post-init sanity check
    if (_strands[strandIdx].dotStarPtr->numPixels() == 0)
      is_success = false;
    WS_DEBUG_PRINT("Created DotStar strand of length ");
    WS_DEBUG_PRINT(pixelsCreateReqMsg->pixels_num);
    WS_DEBUG_PRINT(" on Data GPIO #");
    WS_DEBUG_PRINTLN(pixelsCreateReqMsg->pixels_pin_dotstar_data);
  }

  // fill `wippersnapper_pixels_v1_PixelsCreateResponse` message
  size_t msgSz; // message's encoded size
  wippersnapper_signal_v1_PixelsResponse msgInitResp =
      wippersnapper_signal_v1_PixelsResponse_init_zero;
  msgInitResp.which_payload =
      wippersnapper_signal_v1_PixelsResponse_resp_pixels_create_tag;
  msgInitResp.payload.resp_pixels_create.is_success = is_success;

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

/**************************************************************************/
/*!
    @brief   Obtains the index of a `strand_t` within array of `_strands`.
    @param   dataPin
             strand_t's data dataPin
    @param   type
             Type of strand_t, NeoPixel or DotStar.
    @returns The index of a strand_t if within strands[], -1 otherwise.
*/
/**************************************************************************/
int ws_pixels::getStrandIdx(int16_t dataPin,
                            wippersnapper_pixels_v1_PixelsType type) {
  int strandIdx = -1;

  if (type == wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    for (int i = 0; i < sizeof(_strands); i++) {
      if (_strands[i].pinNeoPixel == dataPin)
        strandIdx = i;
      return strandIdx;
    }
  }
  if (type == wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    for (int i = 0; i < sizeof(_strands); i++) {
      if (_strands[i].pinDotStarData == dataPin)
        strandIdx = i;
      return strandIdx;
    }
  }
  return strandIdx;
}

/**************************************************************************/
/*!
    @brief   Deletes a `strand_t` from `_strands`, deinitializes a strand,
             and frees its resources.
    @param   pixelsDeleteMsg
             Protobuf message from Adafruit IO containing a
             `wippersnapper_pixels_v1_PixelsDeleteRequest`.
*/
/**************************************************************************/
void ws_pixels::deleteStrand(
    wippersnapper_pixels_v1_PixelsDeleteRequest *pixelsDeleteMsg) {
  int strandIdx = getStrandIdx(atoi(pixelsDeleteMsg->pixels_pin_data + 1),
                               pixelsDeleteMsg->pixels_type);
  if (strandIdx == -1) {
    WS_DEBUG_PRINTLN("ERROR: Strand not found, can not delete strand!");
    return;
  }

  // deallocate and release resources of strand object
  deallocateStrand(strandIdx);
}

/**************************************************************************/
/*!
    @brief   Writes a color to a strand_t.
    @param   pixelsWriteMsg
             Protobuf message from Adafruit IO containing data to write.
*/
/**************************************************************************/
void ws_pixels::writeStrand(
    wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg) {
  // Attempt to get strand's index
  int pinData = atoi(pixelsWriteMsg->pixels_pin_data + 1);
  int strandIdx = getStrandIdx(pinData, pixelsWriteMsg->pixels_type);
  if (strandIdx == -1) {
    WS_DEBUG_PRINTLN(
        "ERROR: Strand not found, can not write a color to the strand!");
    return;
  }

  if (pixelsWriteMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    // let's fill the strand
    for (int i = 0; i < _strands[strandIdx].neoPixelPtr->numPixels(); i++) {
      // set color
      _strands[strandIdx].neoPixelPtr->setPixelColor(
          pixelsWriteMsg->pixels_color, i);
    }
    // display color
    _strands[strandIdx].neoPixelPtr->show();
  }

  if (pixelsWriteMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    // let's fill the strand
    for (int i = 0; i < _strands[strandIdx].dotStarPtr->numPixels(); i++) {
      // set color
      _strands[strandIdx].dotStarPtr->setPixelColor(
          pixelsWriteMsg->pixels_color, i);
    }
    // display color
    _strands[strandIdx].dotStarPtr->show();
  }
}
