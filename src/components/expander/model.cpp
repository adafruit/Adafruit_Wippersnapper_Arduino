/*!
 * @file src/components/expander/model.cpp
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
#include "model.h"

ExpanderModel::ExpanderModel() {}

ExpanderModel::~ExpanderModel() {}

/*!
 * @brief Wraps an Added response in a D2B envelope for publishing.
 * @param response The I2C DeviceAddedOrReplaced response.
 * @returns Pointer to the ws_expander_D2B message.
 */
ws_expander_D2B *
ExpanderModel::GetAddedD2B(const ws_i2c_DeviceAddedOrReplaced &response) {
  memset(&_msg_d2b, 0, sizeof(_msg_d2b));
  _msg_d2b.which_payload = ws_expander_D2B_added_tag;
  _msg_d2b.payload.added.has_response = true;
  _msg_d2b.payload.added.response = response;
  return &_msg_d2b;
}

/*!
 * @brief Wraps a Removed response in a D2B envelope for publishing.
 * @param response The I2C DeviceRemoved response.
 * @returns Pointer to the ws_expander_D2B message.
 */
ws_expander_D2B *
ExpanderModel::GetRemovedD2B(const ws_i2c_DeviceRemoved &response) {
  memset(&_msg_d2b, 0, sizeof(_msg_d2b));
  _msg_d2b.which_payload = ws_expander_D2B_removed_tag;
  _msg_d2b.payload.removed.has_response = true;
  _msg_d2b.payload.removed.response = response;
  return &_msg_d2b;
}
