/*!
 * @file src/components/pixels/controller.h
 *
 * Controller for the pixels API
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
#ifndef WS_PIXELS_CONTROLLER_H
#define WS_PIXELS_CONTROLLER_H
#include "wippersnapper.h"
#include "hardware.h"
#include "model.h"

#define MAX_PIXEL_STRANDS                                                      \
  10 ///< Maximum number of pixel strands connected to a WipperSnapper device
#define STRAND_NOT_FOUND 0xFF ///< Strand not found in the array

class wippersnapper; ///< Forward declaration
class PixelsModel;      ///< Forward declaration
class PixelsHardware;   ///< Forward declaration

/*!
    @brief  Routes messages using the pixels.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of pixel strands.
*/
class PixelsController {
public:
  PixelsController();
  ~PixelsController();
  bool Router(pb_istream_t *stream);
  bool Handle_Pixels_Add(ws_pixels_Add *msg);
  bool Handle_Pixels_Write(ws_pixels_Write *msg);
  bool Handle_Pixels_Remove(ws_pixels_Remove *msg);

private:
  PixelsModel *_pixels_model = nullptr; ///< Pointer to the model class
  PixelsHardware *_pixel_strands[MAX_PIXEL_STRANDS] = {
      nullptr};                               ///< Pointer to the hardware class
  uint8_t _num_strands;                       ///< Number of pixel strands
  uint16_t GetStrandIndex(uint16_t pin_data); // Returns 0xFF if not found
};
extern wippersnapper Ws; ///< Global V2 instance
#endif                        // WS_PIXELS_CONTROLLER_H