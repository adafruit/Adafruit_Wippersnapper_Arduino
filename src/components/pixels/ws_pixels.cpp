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

strand_s strands[MAX_PIXEL_STRANDS]{
    nullptr,
    nullptr,
    wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED,
    0,
    0,
    wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_UNSPECIFIED,
    -1,
    -1,
    -1};

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_pixels::~ws_pixels() {
  // de-allocate all strands
  for (int i = 0; i < sizeof(strands) / sizeof(strands[0]); i++)
    deallocateStrand(i);
}

/**************************************************************************/
/*!
    @brief  Deallocates a `strand_t` within `strands`, provided an index.
    @param  strandIdx
            The desired index of a `strand_t` within `strands`.
*/
/**************************************************************************/
void ws_pixels::deallocateStrand(int16_t strandIdx) {
  // reset pixel type
  strands[strandIdx].type =
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED;

  // delete the pixel type object
  if (strands[strandIdx].neoPixelPtr != nullptr)
    delete strands[strandIdx].neoPixelPtr;
  if ((strands[strandIdx].dotStarPtr != nullptr))
    delete strands[strandIdx].dotStarPtr;

  // was this pixel previously used as a status LED?
  if (strands[strandIdx].pinNeoPixel == getStatusNeoPixelPin() ||
      strands[strandIdx].pinDotStarData == getStatusDotStarDataPin())
    initStatusLED();
}

/******************************************************************************/
/*!
    @brief   Allocates an index of a free strand_t within the strand array.
    @returns Index of a free strand_t, -1 if strand array is full.
*/
/******************************************************************************/
int16_t ws_pixels::allocateStrand() {
  for (int16_t strandIdx = 0; strandIdx < sizeof(strands) / sizeof(strands[0]);
       strandIdx++) {
    if (strands[strandIdx].type ==
        wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED) {
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

  // attempt to allocate a free strand from array of strands
  int16_t strandIdx = allocateStrand();
  if (strandIdx == -1)
    is_success = false;

  // TODO: check if is_success == false before going through
  // the init. routine

  if (pixelsCreateReqMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    char *pixelsPin = pixelsCreateReqMsg->pixels_pin_neopixel + 1;
    // is requested pin in-use by the status pixel?
    if (getStatusNeoPixelPin() == atoi(pixelsPin) && WS.lockStatusNeoPixel)
      releaseStatusLED(); // release it!

    // Save data from message into strand structure
    strands[strandIdx].type = pixelsCreateReqMsg->pixels_type;
    strands[strandIdx].pinNeoPixel = atoi(pixelsPin);
    strands[strandIdx].brightness = pixelsCreateReqMsg->pixels_brightness;
    strands[strandIdx].ordering = pixelsCreateReqMsg->pixels_ordering;
    strands[strandIdx].numPixels = pixelsCreateReqMsg->pixels_num;
    // TODO ^ Implement this elsewhere in the code!!
    // Create a new strand of NeoPixels
    strands[strandIdx].neoPixelPtr = new Adafruit_NeoPixel(
        pixelsCreateReqMsg->pixels_num, atoi(pixelsPin),
        getNeoPixelStrandType(pixelsCreateReqMsg->pixels_ordering));
    // Initialize strand
    strands[strandIdx].neoPixelPtr->begin();
    strands[strandIdx].neoPixelPtr->setBrightness(
        strands[strandIdx].brightness);
    strands[strandIdx].neoPixelPtr->clear();
    strands[strandIdx].neoPixelPtr->show();
    // Check that we've correctly initialized the strand
    if (strands[strandIdx].neoPixelPtr->numPixels() == 0)
      is_success = false;

    if (is_success) {
      WS_DEBUG_PRINT("Created NeoPixel strand of length ");
      WS_DEBUG_PRINT(pixelsCreateReqMsg->pixels_num);
      WS_DEBUG_PRINT(" on GPIO #");
      WS_DEBUG_PRINTLN(pixelsCreateReqMsg->pixels_pin_neopixel);
    }
  }

  if (pixelsCreateReqMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    // unpack pins
    char *pinData = pixelsCreateReqMsg->pixels_pin_dotstar_data + 1;
    char *pinClock = pixelsCreateReqMsg->pixels_pin_dotstar_clock + 1;
    // fill strand_t with fields from `pixelsCreateReqMsg`
    strands[strandIdx].type = pixelsCreateReqMsg->pixels_type;
    strands[strandIdx].brightness = pixelsCreateReqMsg->pixels_brightness;
    strands[strandIdx].ordering = pixelsCreateReqMsg->pixels_ordering;
    strands[strandIdx].pinDotStarData = atoi(pinData);
    strands[strandIdx].pinDotStarClock = atoi(pinClock);
    // release the status dotstar, if it is both in-use and the pin within
    // `pixelsCreateReqMsg`
    if ((strands[strandIdx].pinDotStarData == getStatusDotStarDataPin()) &&
        WS.lockStatusDotStar)
      releaseStatusLED();
    // init DotStar object
    strands[strandIdx].dotStarPtr = new Adafruit_DotStar(
        pixelsCreateReqMsg->pixels_num, strands[strandIdx].pinDotStarData,
        strands[strandIdx].pinDotStarClock,
        getDotStarStrandOrder(pixelsCreateReqMsg->pixels_ordering));
    // init. strand for output
    strands[strandIdx].dotStarPtr->begin();
    strands[strandIdx].dotStarPtr->setBrightness(strands[strandIdx].brightness);
    strands[strandIdx].dotStarPtr->clear();
    strands[strandIdx].dotStarPtr->show();

    // post-init sanity check
    if (strands[strandIdx].dotStarPtr->numPixels() == 0)
      is_success = false;
    WS_DEBUG_PRINT("Created DotStar strand of length ");
    WS_DEBUG_PRINT(pixelsCreateReqMsg->pixels_num);
    WS_DEBUG_PRINT(" on Data GPIO #");
    WS_DEBUG_PRINTLN(pixelsCreateReqMsg->pixels_pin_dotstar_data);
  }

  // create `wippersnapper_pixels_v1_PixelsCreateResponse` message
  size_t msgSz; // message's encoded size
  wippersnapper_signal_v1_PixelsResponse msgInitResp =
      wippersnapper_signal_v1_PixelsResponse_init_zero;
  // fill `wippersnapper_pixels_v1_PixelsCreateResponse` message
  msgInitResp.which_payload =
      wippersnapper_signal_v1_PixelsResponse_resp_pixels_create_tag;
  msgInitResp.payload.resp_pixels_create.is_success = is_success;
  // TODO: This should handle the dotstar data pin as well as neopixel data pin
  memcpy(msgInitResp.payload.resp_pixels_create.pixels_pin_data,
         pixelsCreateReqMsg->pixels_pin_neopixel, sizeof(char) * 6);

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
    @brief   Obtains the index of a `strand_t` within array of `strands`.
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
    for (int i = 0; i < sizeof(strands); i++) {
      if (strands[i].pinNeoPixel == dataPin)
        return i;
    }
  }
  if (type == wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    for (int i = 0; i < sizeof(strands); i++) {
      if (strands[i].pinDotStarData == dataPin)
        strandIdx = i;
      return strandIdx;
    }
  }
  return strandIdx;
}

/**************************************************************************/
/*!
    @brief   Deletes a `strand_t` from `strands`, deinitializes a strand,
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

void ws_pixels::writeStrandNeoPixel(
    wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg) {
  // Obtain index of pixel strand
  int strandIdx = getStrandIdx(atoi(pixelsWriteMsg->pixels_pin_data + 1),
                               pixelsWriteMsg->pixels_type);
  if (strandIdx == -1) {
    WS_DEBUG_PRINTLN(
        "ERROR: Pixel strand not found, can not write a color to the strand!");
    return;
  }

  // Fill color from Adafruit IO to the strand
  for (int i = 0; i < strands[strandIdx].numPixels; i++) {
    uint32_t rgbcolorGamma =
        strands[strandIdx].neoPixelPtr->gamma32(pixelsWriteMsg->pixels_color);
    strands[strandIdx].neoPixelPtr->setPixelColor(i, rgbcolorGamma);
  }

  // Display the color on the strand
  strands[strandIdx].neoPixelPtr->show();
}

void ws_pixels::writeStrandDotStar(
    wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg) {
  // TODO!
}