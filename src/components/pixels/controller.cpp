/*!
 * @file controller.cpp
 *
 * Implementation for the pixels API controller.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

/**************************************************************************/
/*!
    @brief  Constructs a new PixelsController object
*/
/**************************************************************************/
PixelsController::PixelsController() {
  _pixels_model = new PixelsModel();
  _num_strands = 0;
}

/**************************************************************************/
/*!
    @brief  Destructs a PixelsController object
*/
/**************************************************************************/
PixelsController::~PixelsController() {
  delete _pixels_model;
  delete _pixel_strands;
  _num_strands = 0;
}

/**************************************************************************/
/*!
    @brief  Handles a request to add a pixel strand
    @param  stream
            Protocol buffer input stream
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool PixelsController::Handle_Pixels_Add(pb_istream_t *stream) {
  // Attempt to decode the istream
  if (!_pixels_model->DecodePixelsAdd(stream))
    return false;
  // Get the decoded message
  wippersnapper_pixels_PixelsAdd *msg_add = _pixels_model->GetPixelsAddMsg();
  _pixel_strands[_num_strands] = new PixelsHardware();

  // TODO: Call ConfigureStrand()!
  bool did_init = false;
  did_init = _pixel_strands[_num_strands]->ConfigureStrand(
      msg_add->pixels_type, msg_add->pixels_ordering, msg_add->pixels_num,
      msg_add->pixels_brightness, msg_add->pixels_pin_data,
      msg_add->pixels_pin_dotstar_clock);
  if (!did_init)
    WS_DEBUG_PRINTLN("[pixels] Failed to create strand!");
  // TODO: Implement the wippersnapper_pixels_PixelsAdded message back to the
  // broker here
}

/**************************************************************************/
/*!
    @brief  Handles a request to write to a pixel strand
    @param  stream
            Protocol buffer input stream
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool PixelsController::Handle_Pixels_Write(pb_istream_t *stream) {}

/**************************************************************************/
/*!
    @brief  Handles a request to remove a pixel strand
    @param  stream
            Protocol buffer input stream
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool PixelsController::Handle_Pixels_Remove(pb_istream_t *stream) {}
