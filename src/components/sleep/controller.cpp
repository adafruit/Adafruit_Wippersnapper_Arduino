/*!
 * @file src/components/sleep/controller.cpp
 *
 * Controller for the sleep.proto API
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
#ifdef ARDUINO_ARCH_ESP32
#include "controller.h"

/*!
    @brief  Sleep controller constructor
*/
SleepController::SleepController() {
  _sleep_hardware = new SleepHardware();
  _sleep_model = new SleepModel();
  _lock = false; // Class-level lock
  _btn_cfg_mode = _sleep_hardware->CheckBootButton();
}

/*!
    @brief  Sleep controller destructor
*/
SleepController::~SleepController() {
  if (_sleep_hardware)
    delete _sleep_hardware;
  if (_sleep_model)
    delete _sleep_model;
}

/*!
    @brief  Routes messages using the sleep.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool SleepController::Router(pb_istream_t *stream) {
  // Attempt to decode the Sleep B2D envelope
  ws_sleep_B2D b2d = ws_sleep_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_sleep_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Unable to decode Sleep B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_sleep_B2D_enter_tag:
    res = Handle_Sleep_Enter(&b2d.payload.enter);
    break;
  default:
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unsupported Sleep payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles a Sleep Enter message from the broker.
    @param  msg
            The Sleep Enter message.
    @return True if the sleep mode was successfully entered, False otherwise.
*/
bool SleepController::Handle_Sleep_Enter(ws_sleep_Enter *msg) {
  WS_DEBUG_PRINTLN("[sleep] Handle_Sleep_Enter()");

  // Parse and handle lock state
  _lock = msg->lock;
  // If boot button was pressed, override any existing lock
  // TODO: Area to refactor
  if (_btn_cfg_mode && _lock) {
    WS_DEBUG_PRINTLN("[sleep] Boot button pressed during startup - overriding lock state");
    _lock = false;
  }

  if (! _lock) {
    WS_DEBUG_PRINTLN("[sleep] Unlocked device from sleep, resuming regular operation.");
    return true;
  }


  return true;
}

/*!
    @brief  Gets the internal SleepModel instance.
    @return Pointer to the internal SleepModel.
*/
SleepModel *SleepController::GetModel() { return _sleep_model; }

/*!
    @brief  Configures deep sleep with a timer wakeup source.
    @param  wakeup
            
*/
bool SleepController::ConfigureDeepSleep(ws_sleep_TimerConfig cfg_timer) {
    // TODO
}

/*!
    @brief  Configures deep sleep with a pin wakeup source.
*/
bool SleepController::ConfigureDeepSleep(ws_sleep_PinConfig cfg_pin) {
    // TODO
}

#endif // ARDUINO_ARCH_ESP32