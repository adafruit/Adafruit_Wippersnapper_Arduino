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
  for (int i = 0; i < _num_strands; i++) {
    delete _pixel_strands[i];
  }
  _num_strands = 0;
  delete _pixels_model;
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
  // Attempt to decode the istream into a PixelsAdd message
  if (!_pixels_model->DecodePixelsAdd(stream))
    return false;
  wippersnapper_pixels_PixelsAdd *msg_add = _pixels_model->GetPixelsAddMsg();
  _pixel_strands[_num_strands] = new PixelsHardware();

  // Configure the pixel strand
  bool did_init = false;
  did_init = _pixel_strands[_num_strands]->AddStrand(
      msg_add->pixels_type, msg_add->pixels_ordering, msg_add->pixels_num,
      msg_add->pixels_brightness, msg_add->pixels_pin_data,
      msg_add->pixels_pin_dotstar_clock);
  if (!did_init)
    WS_DEBUG_PRINTLN("[pixels] Failed to create strand!");

  // Publish PixelsAdded message to the broker
  if (!_pixels_model->EncodePixelsAdded(msg_add->pixels_pin_data, did_init)) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to encode PixelsAdded message!");
    return false;
  }
  if (!WsV2.PublishSignal(wippersnapper_signal_DeviceToBroker_pixels_added_tag,
                          _pixels_model->GetPixelsAddedMsg())) {
    WS_DEBUG_PRINTLN("[pixels]: Unable to publish PixelsAdded message!");
    return false;
  }

  return true;
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
  // Decode the PixelsWrite message
  if (!_pixels_model->DecodePixelsWrite(stream)) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to decode PixelsWrite message!");
    return false;
  }
  wippersnapper_pixels_PixelsWrite *msg_write =
      _pixels_model->GetPixelsWriteMsg();
  uint16_t pin_data = atoi(msg_write->pixels_pin_data + 1);
  uint16_t idx = GetStrandIndex(pin_data);
  if (idx == -1) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to find strand index!");
    return false;
  }

  // Call hardware to fill the strand
  _pixel_strands[idx]->FillStrand(msg_write->pixels_color);
  return true;
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
  // Decode the PixelsRemove message
  if (!_pixels_model->DecodePixelsRemove(stream)) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to decode PixelsRemove message!");
    return false;
  }
  wippersnapper_pixels_PixelsRemove *msg_remove =
      _pixels_model->GetPixelsRemoveMsg();

  uint16_t pin_data = atoi(msg_remove->pixels_pin_data + 1);
  uint16_t idx = GetStrandIndex(pin_data);
  if (idx == -1) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to find strand index!");
    return false;
  }

  // Call hardware to deinitialize the strand
  _pixel_strands[idx]->RemoveStrand();
  return true;
}

/**************************************************************************/
/*!
    @brief  Handles a request to update the pixel strands
    @param  pin_data
            The desired data pin
    @returns Desired strand index, -1 if not found.
*/
/**************************************************************************/
uint16_t PixelsController::GetStrandIndex(uint16_t pin_data) {
  for (int i = 0; i < _num_strands; i++) {
    if (_pixel_strands[i]->GetPinData() == pin_data) {
      return i;
    }
  }
  return -1;
}