/*!
 * @file ws_pixels.cpp
 *
 * High-level interface for wippersnapper to manage addressable RGB pixel
 * strands
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * Written by Brent Rubell for Adafruit Industries, 2022-2023
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
    -1}; ///< Contains all pixel strands used by WipperSnapper

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_pixels::~ws_pixels() {
  // de-allocate all strands
  for (size_t i = 0; i < sizeof(strands) / sizeof(strands[0]); i++)
    deallocateStrand(i);
}

/******************************************************************************/
/*!
    @brief   Allocates an index of a free strand_t within the strand array.
    @returns Index of a free strand_t, ERR_INVALID_STRAND if strand array is
   full.
*/
/******************************************************************************/
int16_t ws_pixels::allocateStrand() {
  for (size_t strandIdx = 0; strandIdx < sizeof(strands) / sizeof(strands[0]);
       strandIdx++) {
    if (strands[strandIdx].type ==
        wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED) {
      return strandIdx;
    }
  }
  return ERR_INVALID_STRAND;
}

/**************************************************************************/
/*!
    @brief  Deallocates a `strand_t` within `strands`, provided an index.
    @param  strandIdx
            The desired index of a `strand_t` within `strands`.
*/
/**************************************************************************/
void ws_pixels::deallocateStrand(int16_t strandIdx) {
  // delete the pixel object
  if (strands[strandIdx].neoPixelPtr != nullptr) {
    // Fill with "off"
    strands[strandIdx].neoPixelPtr->clear();
    strands[strandIdx].neoPixelPtr->show();
    // Delete the NeoPixel object
    delete strands[strandIdx].neoPixelPtr;
  } else if ((strands[strandIdx].dotStarPtr != nullptr)) {
    // Fill with "off"
    strands[strandIdx].dotStarPtr->clear();
    strands[strandIdx].dotStarPtr->show();
    delete strands[strandIdx].dotStarPtr;
  }

  // re-initialize status pixel (if pixel was prvsly used)
  if (strands[strandIdx].pinNeoPixel == getStatusNeoPixelPin() ||
      strands[strandIdx].pinDotStarData == getStatusDotStarDataPin()) {
    initStatusLED();
  }

  // reset the strand
  strands[strandIdx] = {
      nullptr,
      nullptr,
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_UNSPECIFIED,
      0,
      0,
      wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_UNSPECIFIED,
      -1,
      -1,
      -1};
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
neoPixelType ws_pixels::getNeoPixelStrandOrder(
    wippersnapper_pixels_v1_PixelsOrder pixelOrder) {
  switch (pixelOrder) {
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GRB:
    return NEO_GRB + NEO_KHZ800;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GRBW:
    return NEO_GRBW + NEO_KHZ800;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RGB:
    return NEO_RGB + NEO_KHZ800;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RGBW:
    return NEO_RGBW + NEO_KHZ800;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_BRG:
    return NEO_BRG + NEO_KHZ800;
  default:
    return NEO_GRB + NEO_KHZ800;
  }
}

/**************************************************************************/
/*!
    @brief   Returns the color order of the DotStar strand.
    @param   pixelOrder
             Desired pixel order, from init. message.
    @returns Type of DotStar strand.
*/
/**************************************************************************/
uint8_t ws_pixels::getDotStarStrandOrder(
    wippersnapper_pixels_v1_PixelsOrder pixelOrder) {
  switch (pixelOrder) {
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GRB:
    return DOTSTAR_GRB;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RGB:
    return DOTSTAR_RGB;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_BRG:
    return DOTSTAR_BRG;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_RBG:
    return DOTSTAR_RBG;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_GBR:
    return DOTSTAR_GBR;
  case wippersnapper_pixels_v1_PixelsOrder_PIXELS_ORDER_BGR:
    return DOTSTAR_BGR;
  default:
    return DOTSTAR_BRG;
  }
}

/**************************************************************************/
/*!
    @brief   Creates a PixelsResponse message and publishes it to IO.
    @param   is_success
             True if `addStrand()` succeeded, False otherwise.
    @param   pixels_pin_data
             The strand's data pin..
*/
/**************************************************************************/
void ws_pixels::publishAddStrandResponse(bool is_success,
                                         char *pixels_pin_data) {
  // Create response message
  wippersnapper_signal_v1_PixelsResponse msgInitResp =
      wippersnapper_signal_v1_PixelsResponse_init_zero;
  msgInitResp.which_payload =
      wippersnapper_signal_v1_PixelsResponse_resp_pixels_create_tag;
  // Fill response message
  msgInitResp.payload.resp_pixels_create.is_success = is_success;
  memcpy(msgInitResp.payload.resp_pixels_create.pixels_pin_data,
         pixels_pin_data, sizeof(char) * 6);

  // Encode `wippersnapper_pixels_v1_PixelsCreateResponse` message
  memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
  pb_ostream_t ostream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!ws_pb_encode(&ostream, wippersnapper_signal_v1_PixelsResponse_fields,
                    &msgInitResp)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode "
                     "wippersnapper_signal_v1_PixelsResponse message!");
    return;
  }

  // Publish message to broker
  size_t msgSz;
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_PixelsResponse_fields,
                      &msgInitResp);
  WS_DEBUG_PRINT("-> wippersnapper_signal_v1_PixelsResponse...");
  WS._mqtt->publish(WS._topic_signal_pixels_device, WS._buffer_outgoing, msgSz,
                    1);
  WS_DEBUG_PRINTLN("Published!");
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
  // attempt to allocate a free strand from array of strands
  int16_t strandIdx = allocateStrand();
  if (strandIdx == ERR_INVALID_STRAND) { // unable to allocate a strand
    if (pixelsCreateReqMsg->pixels_type ==
        wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL)
      publishAddStrandResponse(false, pixelsCreateReqMsg->pixels_pin_neopixel);
    else if (pixelsCreateReqMsg->pixels_type ==
             wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR)
      publishAddStrandResponse(false,
                               pixelsCreateReqMsg->pixels_pin_dotstar_data);
    return false;
  }

  // fill generic members of the strand obj.
  strands[strandIdx].type = pixelsCreateReqMsg->pixels_type;
  strands[strandIdx].brightness = pixelsCreateReqMsg->pixels_brightness;
  strands[strandIdx].numPixels = pixelsCreateReqMsg->pixels_num;
  strands[strandIdx].ordering = pixelsCreateReqMsg->pixels_ordering;

  // fill strand pins
  if (pixelsCreateReqMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    strands[strandIdx].pinNeoPixel =
        atoi(pixelsCreateReqMsg->pixels_pin_neopixel + 1);
  } else if (pixelsCreateReqMsg->pixels_type ==
             wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    strands[strandIdx].pinDotStarData =
        atoi(pixelsCreateReqMsg->pixels_pin_dotstar_data + 1);
    strands[strandIdx].pinDotStarClock =
        atoi(pixelsCreateReqMsg->pixels_pin_dotstar_clock + 1);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid strand type provided!");
    publishAddStrandResponse(false, pixelsCreateReqMsg->pixels_pin_neopixel);
    return false;
  }

  // Fill specific members of strand obj.
  if (pixelsCreateReqMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    // Release status LED
    // is requested pin in-use by the status pixel?
    if (getStatusNeoPixelPin() == strands[strandIdx].pinNeoPixel &&
        WS.lockStatusNeoPixel)
      releaseStatusLED(); // release it!

    // Create a new strand of NeoPixels
    WS_DEBUG_PRINTLN("Setting up new NeoPixel Strand...");
    strands[strandIdx].neoPixelPtr = new Adafruit_NeoPixel(
        pixelsCreateReqMsg->pixels_num, strands[strandIdx].pinNeoPixel,
        getNeoPixelStrandOrder(pixelsCreateReqMsg->pixels_ordering));

    // Initialize strand
    strands[strandIdx].neoPixelPtr->begin();
    strands[strandIdx].neoPixelPtr->setBrightness(
        strands[strandIdx].brightness);
    strands[strandIdx].neoPixelPtr->clear();
    strands[strandIdx].neoPixelPtr->show();

    // Check that we've correctly initialized the strand
    if (strands[strandIdx].neoPixelPtr->numPixels() == 0) {
      publishAddStrandResponse(false, pixelsCreateReqMsg->pixels_pin_neopixel);
      return false;
    }

    WS_DEBUG_PRINT("Created NeoPixel strand of length ");
    WS_DEBUG_PRINTVAR(pixelsCreateReqMsg->pixels_num);
    WS_DEBUG_PRINT(" on GPIO #");
    WS_DEBUG_PRINTLNVAR(pixelsCreateReqMsg->pixels_pin_neopixel);
    publishAddStrandResponse(true, pixelsCreateReqMsg->pixels_pin_neopixel);
  } else if (pixelsCreateReqMsg->pixels_type ==
             wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {

    // release the status dotstar, if it is both in-use and the pin within
    // `pixelsCreateReqMsg`
    if ((strands[strandIdx].pinDotStarData == getStatusDotStarDataPin()) &&
        WS.lockStatusDotStar) {
      releaseStatusLED();
    }

    // Create Dotstar strand
    strands[strandIdx].dotStarPtr = new Adafruit_DotStar(
        strands[strandIdx].numPixels, strands[strandIdx].pinDotStarData,
        strands[strandIdx].pinDotStarClock,
        getDotStarStrandOrder(strands[strandIdx].ordering));

    // initialize strand
    strands[strandIdx].dotStarPtr->begin();
    strands[strandIdx].dotStarPtr->setBrightness(strands[strandIdx].brightness);
    strands[strandIdx].dotStarPtr->clear();
    strands[strandIdx].dotStarPtr->show();

    // post-init sanity check
    if (strands[strandIdx].dotStarPtr->numPixels() == 0) {
      publishAddStrandResponse(false,
                               pixelsCreateReqMsg->pixels_pin_dotstar_data);
      return false;
    }

    WS_DEBUG_PRINT("Created DotStar strand of length ");
    WS_DEBUG_PRINTVAR(strands[strandIdx].numPixels);
    WS_DEBUG_PRINT(" on Data GPIO #");
    WS_DEBUG_PRINTLNVAR(strands[strandIdx].pinDotStarData);
    publishAddStrandResponse(true, pixelsCreateReqMsg->pixels_pin_dotstar_data);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid strand type provided!");
    publishAddStrandResponse(false,
                             pixelsCreateReqMsg->pixels_pin_dotstar_data);
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief   Obtains the index of a `strand_t` within array of `strands`.
    @param   dataPin
             strand_t's data dataPin
    @param   type
             Type of strand_t, NeoPixel or DotStar.
    @returns The index of a strand_t if within strands[],
             ERR_INVALID_STRAND otherwise.
*/
/**************************************************************************/
int ws_pixels::getStrandIdx(int16_t dataPin,
                            wippersnapper_pixels_v1_PixelsType type) {
  for (size_t strandIdx = 0; strandIdx < sizeof(strands) / sizeof(strands[0]);
       strandIdx++) {
    if (type == wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL &&
        strands[strandIdx].pinNeoPixel == dataPin)
      return strandIdx;
    if (type == wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR &&
        strands[strandIdx].pinDotStarData == dataPin)
      return strandIdx;
  }
  return ERR_INVALID_STRAND;
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
  if (strandIdx == ERR_INVALID_STRAND) {
    WS_DEBUG_PRINTLN("ERROR: Strand not found, unable to delete strand!");
    return;
  }
  // deallocate and release resources of strand object
  deallocateStrand(strandIdx);

  WS_DEBUG_PRINT("Deleted strand on data pin ");
  WS_DEBUG_PRINTLNVAR(pixelsDeleteMsg->pixels_pin_data);
}

/**************************************************************************/
/*!
    @brief   Gets the gamma-corrected color, provided a pixel_color
    @param   pixel_color
             Strand's color from Adafruit IO.
    @param   strand
             Desired strand struct. to access.
    @returns A gamma-corrected strand color
*/
/**************************************************************************/
uint32_t ws_pixels::getGammaCorrectedColor(uint32_t pixel_color,
                                           strand_s strand) {
  if (strand.neoPixelPtr != nullptr) {
    return strand.neoPixelPtr->gamma32(pixel_color);
  } else if (strand.dotStarPtr != nullptr) {
    return strand.dotStarPtr->gamma32(pixel_color);
  } else {
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to perform gamma correction, unknown strand type!");
    return 0;
  }
}

/**************************************************************************/
/*!
    @brief   Writes a color from Adafruit IO to a strand of
             addressable pixels
    @param   pixelsWriteMsg
             Protobuf message from Adafruit IO containing a
             `wippersnapper_pixels_v1_PixelsWriteRequest`.
*/
/**************************************************************************/
void ws_pixels::fillStrand(
    wippersnapper_pixels_v1_PixelsWriteRequest *pixelsWriteMsg) {

  // Get index of pixel strand
  int strandIdx = getStrandIdx(atoi(pixelsWriteMsg->pixels_pin_data + 1),
                               pixelsWriteMsg->pixels_type);
  if (strandIdx == ERR_INVALID_STRAND) {
    WS_DEBUG_PRINTLN(
        "ERROR: Pixel strand not found, can not write a color to the strand!");
    return;
  }

  uint32_t rgbColorGamma =
      getGammaCorrectedColor(pixelsWriteMsg->pixels_color, strands[strandIdx]);

  WS_DEBUG_PRINT("Filling color: ");
  WS_DEBUG_PRINTLNVAR(pixelsWriteMsg->pixels_color);

  if (pixelsWriteMsg->pixels_type ==
      wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_NEOPIXEL) {
    strands[strandIdx].neoPixelPtr->fill(rgbColorGamma);
    strands[strandIdx].neoPixelPtr->show();
  } else if (pixelsWriteMsg->pixels_type ==
             wippersnapper_pixels_v1_PixelsType_PIXELS_TYPE_DOTSTAR) {
    strands[strandIdx].dotStarPtr->fill(rgbColorGamma);
    strands[strandIdx].dotStarPtr->show();
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to determine pixel type to write to!");
  }
}