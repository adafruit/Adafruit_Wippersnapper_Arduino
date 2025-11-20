/*!
 * @file src/components/pixels/model.cpp
 *
 * Implementation for the pixels.proto message model.
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
#include "model.h"
#include "Wippersnapper_V2.h"
#include "nanopb/ws_pb_helpers.h"

/*!
    @brief  Constructs a new PixelsModel object
*/
PixelsModel::PixelsModel() {
  memset(&_msg_pixels_add, 0, sizeof(_msg_pixels_add));
  memset(&_msg_pixels_remove, 0, sizeof(_msg_pixels_remove));
  memset(&_msg_pixels_write, 0, sizeof(_msg_pixels_write));
  memset(&_msg_pixels_added, 0, sizeof(_msg_pixels_added));
  // no-op
}

/*!
    @brief  Destructs a PixelsModel object
*/
PixelsModel::~PixelsModel() {
  memset(&_msg_pixels_add, 0, sizeof(_msg_pixels_add));
  memset(&_msg_pixels_remove, 0, sizeof(_msg_pixels_remove));
  memset(&_msg_pixels_write, 0, sizeof(_msg_pixels_write));
  memset(&_msg_pixels_added, 0, sizeof(_msg_pixels_added));
}

/*!
    @brief  Decodes a PixelsAdd message from a protocol buffer input stream.
    @param  stream
            Protocol buffer input stream.
    @returns True if successful, False otherwise.
*/
bool PixelsModel::DecodePixelsAdd(pb_istream_t *stream) {
  memset(&_msg_pixels_add, 0, sizeof(_msg_pixels_add));
  return pb_decode(stream, ws_pixels_Add_fields, &_msg_pixels_add);
}

/*!
    @brief  Returns a pointer to the PixelsAdd message.
    @returns Pointer to the PixelsAdd message object.
*/
ws_pixels_Add *PixelsModel::GetPixelsAddMsg() { return &_msg_pixels_add; }

/*!
    @brief  Decodes a PixelsRemove message from a protocol buffer input stream.
    @param  stream
            Protocol buffer input stream.
    @returns True if successful, False otherwise.
*/
bool PixelsModel::DecodePixelsRemove(pb_istream_t *stream) {
  memset(&_msg_pixels_remove, 0, sizeof(_msg_pixels_remove));
  return pb_decode(stream, ws_pixels_Remove_fields, &_msg_pixels_remove);
}

/*!
    @brief  Returns a pointer to the PixelsRemove message.
    @returns Pointer to the PixelsRemove message object.
*/
ws_pixels_Remove *PixelsModel::GetPixelsRemoveMsg() {
  return &_msg_pixels_remove;
}

/*!
    @brief  Decodes a PixelsWrite message from a protocol buffer input stream.
    @param  stream
            Protocol buffer input stream.
    @returns True if successful, False otherwise.
*/
bool PixelsModel::DecodePixelsWrite(pb_istream_t *stream) {
  memset(&_msg_pixels_write, 0, sizeof(_msg_pixels_write));
  WS_DEBUG_PRINTLN("Decoding PixelsWrite message...");
  return pb_decode(stream, ws_pixels_Write_fields, &_msg_pixels_write);
}

/*!
    @brief  Returns a pointer to the PixelsWrite message.
    @returns Pointer to the PixelsWrite message object.
*/
ws_pixels_Write *PixelsModel::GetPixelsWriteMsg() { return &_msg_pixels_write; }

/*!
    @brief  Encodes a PixelsAdded message.
    @param  pin_data
            The pin the pixels strand is connected to.
    @param  success
            True if strand was successfully initialized, False otherwise.
    @returns True if successful, False otherwise.
*/
bool PixelsModel::EncodePixelsAdded(char *pin_data, bool success) {
  // Fill the message
  memset(&_msg_pixels_added, 0, sizeof(_msg_pixels_added));
  _msg_pixels_added.is_success = success;
  strncpy(_msg_pixels_added.pin_data, pin_data,
          sizeof(_msg_pixels_added.pin_data));

  // Encode it!
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, ws_pixels_Added_fields, &_msg_pixels_added))
    return false;
  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_pixels_Added_fields, &_msg_pixels_added);
}

/*!
    @brief  Returns a pointer to the PixelsAdded message.
    @returns Pointer to the PixelsAdded message object.
*/
ws_pixels_Added *PixelsModel::GetPixelsAddedMsg() { return &_msg_pixels_added; }