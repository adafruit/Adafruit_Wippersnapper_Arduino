/*!
 * @file controller.h
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
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"

#define MAX_PIXEL_STRANDS 5 ///< Maximum number of pixel strands connected to a WipperSnapper device

class Wippersnapper_V2; ///< Forward declaration
class PixelsModel;     ///< Forward declaration
class PixelsHardware;  ///< Forward declaration

/**************************************************************************/
/*!
    @brief  Routes messages using the pixels.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of pixel strands.
*/
/**************************************************************************/
class PixelsController {
public:
  PixelsController();
  ~PixelsController();
  // Called by the cbDecodeBrokerToDevice router function
  bool Handle_Pixels_Add(pb_istream_t *stream);
  bool Handle_Pixels_Write(pb_istream_t *stream);
  bool Handle_Pixels_Remove(pb_istream_t *stream);
private:
  PixelsModel *_pixels_model = nullptr; ///< Pointer to the model class
  PixelsHardware *_pixel_strands[MAX_PIXEL_STRANDS] = {nullptr}; ///< Pointer to the hardware class
  uint8_t _num_strands; ///< Number of pixel strands
  uint16_t GetStrandIndex(uint16_t pin_data);
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif // WS_PIXELS_CONTROLLER_H