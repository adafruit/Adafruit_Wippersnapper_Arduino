/*!
 * @file src/components/expander/model.h
 *
 * Model interface for the expander.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_EXPANDER_MODEL_H
#define WS_EXPANDER_MODEL_H
#include "wippersnapper.h"
#include <protos/expander.pb.h>

/*!
    @brief  Provides an interface for creating and encoding
            D2B messages from expander.proto.
*/
class ExpanderModel {
public:
  ExpanderModel();
  ~ExpanderModel();
  ws_expander_D2B *GetAddedD2B(const ws_i2c_DeviceAddedOrReplaced &response);
  ws_expander_D2B *GetRemovedD2B(const ws_i2c_DeviceRemoved &response);

private:
  ws_expander_D2B _msg_d2b; ///< D2B envelope for publishing
};
#endif // WS_EXPANDER_MODEL_H
