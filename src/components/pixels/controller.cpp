/*!
 * @file src/components/pixels/controller.cpp
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

/*!
    @brief  Constructs a new PixelsController object
*/
PixelsController::PixelsController() {
  _pixels_model = new PixelsModel();
  _num_strands = 0;
}

/*!
    @brief  Destructs a PixelsController object
*/
PixelsController::~PixelsController() {
  for (int i = 0; i < _num_strands; i++) {
    delete _pixel_strands[i];
  }
  _num_strands = 0;
  delete _pixels_model;
}

/*!
    @brief  Routes messages using the pixels.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool PixelsController::Router(pb_istream_t *stream) {
  // Attempt to decode the Pixels B2D envelope
  ws_pixels_B2D b2d = ws_pixels_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_pixels_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[pixels] ERROR: Unable to decode Pixels B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_pixels_B2D_add_tag:
    res = Handle_Pixels_Add(&b2d.payload.add);
    break;
  case ws_pixels_B2D_remove_tag:
    res = Handle_Pixels_Remove(&b2d.payload.remove);
    break;
  case ws_pixels_B2D_write_tag:
    res = Handle_Pixels_Write(&b2d.payload.write);
    break;
  default:
    WS_DEBUG_PRINTLN("[pixels] WARNING: Unsupported Pixels payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles a request to add a pixel strand
    @param  msg
            The PixelsAdd message.
    @returns True if successful, False otherwise
*/
bool PixelsController::Handle_Pixels_Add(ws_pixels_Add *msg) {
  _pixel_strands[_num_strands] = new PixelsHardware();

  // Configure the pixel strand
  bool did_init = false;
  did_init = _pixel_strands[_num_strands]->AddStrand(
      msg->type, msg->ordering, msg->num, msg->brightness, msg->pin_data,
      msg->pin_dotstar_clock);
  if (!did_init) {
    WS_DEBUG_PRINTLN("[pixels] Failed to create strand!");
  } else {
    _num_strands++;
    WS_DEBUG_PRINT("[pixels]: Added strand #");
    WS_DEBUG_PRINTLN(_num_strands);
  }

  // Publish PixelsAdded message to the broker
  if (!_pixels_model->EncodePixelsAdded(msg->pin_data, did_init)) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to encode PixelsAdded message!");
    return false;
  }
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_pixels_tag,
                     _pixels_model->GetPixelsAddedMsg())) {
    WS_DEBUG_PRINTLN("[pixels]: Unable to publish PixelsAdded message!");
    return false;
  }

  return true;
}

/*!
    @brief  Handles a request to write to a pixel strand
    @param  msg
            The PixelsWrite message.
    @returns True if successful, False otherwise
*/
bool PixelsController::Handle_Pixels_Write(ws_pixels_Write *msg) {
  uint16_t pin_data = atoi(msg->pin_data + 1);
  uint16_t idx = GetStrandIndex(pin_data);
  if (idx == STRAND_NOT_FOUND) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to find strand index!");
    return false;
  }

  // Call hardware to fill the strand
  WS_DEBUG_PRINTLN("[pixels]: Filling strand!");
  _pixel_strands[idx]->FillStrand(msg->color);
  return true;
}

/*!
    @brief  Handles a request to remove a pixel strand
    @param  msg
            The PixelsRemove message.
    @returns True if successful, False otherwise
*/
bool PixelsController::Handle_Pixels_Remove(ws_pixels_Remove *msg) {
  uint16_t pin_data = atoi(msg->pin_data + 1);
  uint16_t idx = GetStrandIndex(pin_data);
  if (idx == STRAND_NOT_FOUND) {
    WS_DEBUG_PRINTLN("[pixels]: Failed to find strand index!");
    return false;
  }

  // Call hardware to deinitialize the strand
  _pixel_strands[idx]->RemoveStrand();
  return true;
}

/*!
    @brief  Gets the index of a strand by its data pin
    @param  pin_data
            The desired data pin
    @returns Desired strand index, or STRAND_NOT_FOUND if not found.
*/
uint16_t PixelsController::GetStrandIndex(uint16_t pin_data) {
  for (uint8_t i = 0; i < _num_strands; i++) {
    if (_pixel_strands[i]->GetPinData() == pin_data) {
      return i;
    }
  }
  return STRAND_NOT_FOUND; // Sentinel value indicating "not found"
}