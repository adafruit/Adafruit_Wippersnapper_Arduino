/*!
 * @file src/components/pixels/model.h
 *
 * Model for the pixels.proto message.
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
#ifndef WS_PIXELS_MODEL_H
#define WS_PIXELS_MODEL_H
#include "Wippersnapper_V2.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from pixels.proto.
*/
class PixelsModel {
public:
  PixelsModel();
  ~PixelsModel();
  // PixelsAdd
  bool DecodePixelsAdd(pb_istream_t *stream);
  ws_pixels_Add *GetPixelsAddMsg();
  // PixelsRemove
  bool DecodePixelsRemove(pb_istream_t *stream);
  ws_pixels_Remove *GetPixelsRemoveMsg();
  // PixelsWrite
  bool DecodePixelsWrite(pb_istream_t *stream);
  ws_pixels_Write *GetPixelsWriteMsg();
  // PixelsAdded
  bool EncodePixelsAdded(char *pin_data, bool success);
  ws_pixels_Added *GetPixelsAddedMsg();

private:
  ws_pixels_Add _msg_pixels_add;       ///< PixelsAdd message object
  ws_pixels_Remove _msg_pixels_remove; ///< PixelsRemove message object
  ws_pixels_Write _msg_pixels_write;   ///< PixelsWrite message object
  ws_pixels_Added _msg_pixels_added;   ///< PixelsAdded message object
};
#endif // WS_PIXELS_MODEL_H