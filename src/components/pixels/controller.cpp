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
}

/**************************************************************************/
/*!
    @brief  Handles a request to write to a pixel strand
    @param  stream
            Protocol buffer input stream
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool PixelsController::Handle_Pixels_Write(pb_istream_t *stream) {
}

/**************************************************************************/
/*!
    @brief  Handles a request to remove a pixel strand
    @param  stream
            Protocol buffer input stream
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool PixelsController::Handle_Pixels_Remove(pb_istream_t *stream) {
}
