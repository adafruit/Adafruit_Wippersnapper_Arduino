/*!
 * @file src/components/pwm/controller.cpp
 *
 * Controller for the pwm API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"
#include "../expander/controller.h"

/*!
    @brief  Ctor for PWMController.
*/
PWMController::PWMController() { _pwm_model = new PWMModel(); }

/*!
    @brief  Dtor for PWMController.
*/
PWMController::~PWMController() {
  for (size_t i = 0; i < _pins.size(); i++)
    delete _pins[i];
  delete _pwm_model;
}

/*!
    @brief  Routes messages using the pwm.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool PWMController::Router(pb_istream_t *stream) {
  // Attempt to decode the PWM B2D envelope
  ws_pwm_B2D b2d = ws_pwm_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_pwm_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[pwm] ERROR: Unable to decode PWM B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_pwm_B2D_add_tag:
    res = Handle_PWM_Add(&b2d.payload.add);
    break;
  case ws_pwm_B2D_remove_tag:
    res = Handle_PWM_Remove(&b2d.payload.remove);
    break;
  case ws_pwm_B2D_write_tag:
    res = Handle_PWM_Write(&b2d.payload.write);
    break;
  default:
    WS_DEBUG_PRINTLN("[pwm] WARNING: Unsupported PWM payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles the PWM_Add message.
    @param  msg The PWMAdd message.
    @return True if the message was handled successfully, false otherwise.
*/
bool PWMController::Handle_PWM_Add(ws_pwm_Add *msg) {
  uint8_t pin = 0;
  if (!ExpanderHardware::ParsePinNum(msg->pin, pin)) {
    Ws.error_handler->publishComponentError(msg->pin, "Malformed pin name");
    return false;
  }

  // Resolve the expander driver if this is an expander pin
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      Ws.error_handler->publishComponentError(msg->pin, "Expander not found");
      return false;
    }
  }

  // If pin already exists, remove it before re-adding (for updates)
  if (GetPin(pin, expander_drv) != nullptr) {
    RemovePin(pin, expander_drv);
  }

  PWMHardware *new_pin = new PWMHardware();
  bool did_attach = new_pin->attach(pin, (uint32_t)msg->frequency,
                                    (uint32_t)msg->resolution, expander_drv);
  if (!did_attach) {
    Ws.error_handler->publishComponentError(msg->pin, "Failed to attach pin");
    delete new_pin;
  } else {
    _pins.push_back(new_pin);
  }

  WS_DEBUG_PRINT("[pwm] Attached pin: ");
  WS_DEBUG_PRINTVAR(msg->pin);
  return true;
}

/*!
    @brief  Handles the PWM_Remove message.
    @param  msg The PWMRemove message.
    @return True if the message was handled successfully, false otherwise.
*/
bool PWMController::Handle_PWM_Remove(ws_pwm_Remove *msg) {
  uint8_t pin = 0;
  if (!ExpanderHardware::ParsePinNum(msg->pin, pin)) {
    Ws.error_handler->publishComponentError(msg->pin, "Malformed pin name");
    return false;
  }

  // Resolve the expander driver if this is an expander pin
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      Ws.error_handler->publishComponentError(msg->pin, "Expander not found");
      return false;
    }
  }

  if (!RemovePin(pin, expander_drv)) {
    Ws.error_handler->publishComponentError(msg->pin, "Failed to find pin");
    return false;
  }

  WS_DEBUG_PRINT("[pwm] Removed pin: ");
  WS_DEBUG_PRINTVAR(msg->pin);
  return true;
}

/*!
    @brief  Removes a pin from the vector by pin number.
            Detaches and deletes the pin object, freeing the hardware resource.
    @param  pin The pin number to remove.
    @return True if the pin was found and removed, False otherwise.
*/
bool PWMController::RemovePin(uint8_t pin, ExpanderHardware *expander) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->GetPin() == pin &&
        _pins[i]->GetExpanderDriver() == expander) {
      _pins[i]->detach();
      delete _pins[i];
      _pins.erase(_pins.begin() + i);
      return true;
    }
  }
  return false;
}

/*!
    @brief  Get a pointer to a PWM pin by pin number.
    @param  pin The pin number to search for.
    @return Pointer to the PWM hardware object, or nullptr if not found.
*/
PWMHardware *PWMController::GetPin(uint8_t pin, ExpanderHardware *expander) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->GetPin() == pin && _pins[i]->GetExpanderDriver() == expander)
      return _pins[i];
  }
  return nullptr;
}

/*!
    @brief  Handles the PWM_Write message.
    @param  msg The PWMWrite message.
    @return True if the message was handled successfully, false otherwise.
*/
bool PWMController::Handle_PWM_Write(ws_pwm_Write *msg) {
  uint8_t pin = 0;
  if (!ExpanderHardware::ParsePinNum(msg->pin, pin)) {
    Ws.error_handler->publishComponentError(msg->pin, "Malformed pin name");
    return false;
  }

  // Resolve the expander driver if this is an expander pin
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      Ws.error_handler->publishComponentError(msg->pin, "Expander not found");
      return false;
    }
  }

  PWMHardware *hw = GetPin(pin, expander_drv);
  if (hw == nullptr) {
    Ws.error_handler->publishComponentError(msg->pin, "Failed to find pin");
    return false;
  }

  if (!hw->write(msg)) {
    Ws.error_handler->publishComponentError(msg->pin,
                                            "Failed to write to pin");
    return false;
  }

  return true;
}