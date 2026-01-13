/*!
 * @file src/components/sleep/controller.cpp
 *
 * Controller for the sleep.proto API
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
#ifdef ARDUINO_ARCH_ESP32
#include "controller.h"

/*!
    @brief  Sleep controller constructor
*/
SleepController::SleepController() {
  _sleep_hardware = new SleepHardware();
  _sleep_model = new SleepModel();
  _sleep_mode = ws_sleep_SleepMode_S_UNSPECIFIED;
  _lock = false; // Class-level lock
  _btn_cfg_mode = _sleep_hardware->CheckBootButton();

// Mark so we can disable all external peripherals that draw power during sleep
// (i.e: tft, i2c, neopixel, etc)
#if defined(NEOPIXEL_POWER) || defined(PIN_I2C_POWER) || defined(TFT_POWER) || \
    defined(TFT_I2C_POWER)
  _has_ext_pwr_components = true;
#else
  _has_ext_pwr_components = false;
#endif
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
    @brief  Gets the internal SleepModel instance.
    @return Pointer to the internal SleepModel.
*/
SleepModel *SleepController::GetModel() { return _sleep_model; }

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
    WS_DEBUG_PRINTLN(
        "[sleep] Boot button pressed during startup - overriding lock state");
    _lock = false;
    return true;
  }

  if (!_lock) {
    WS_DEBUG_PRINTLN(
        "[sleep] Unlocked device from sleep, resuming regular operation.");
    return true;
  }

  // Parse sleep mode and dispatch based on mode
  _sleep_mode = msg->mode;

  // Configure sleep with the provided wakeup configuration
  bool res = false;
  if (_sleep_mode == ws_sleep_SleepMode_S_DEEP || _sleep_mode == ws_sleep_SleepMode_S_LIGHT) {
    res = ConfigureSleep(msg);
  } else {
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unsupported sleep mode");
  }

  return res;
}

/*!
    @brief  Configures the sleep mode based on the provided Enter message.
    @param  msg
            Pointer to the Sleep Enter message containing config union.
    @return True if deep sleep was successfully configured, False otherwise.
*/
bool SleepController::ConfigureSleep(const ws_sleep_Enter *msg) {
  bool rc = false;
  switch (msg->which_config) {
  case ws_sleep_Enter_timer_tag:
    // Set timer-based wakeup
    rc = _sleep_hardware->RegisterRTCTimerWakeup(msg->config.timer.duration * 1000000);
    if (!rc) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to set timer wakeup");
    }
    // TODO: Remove this debug print in production build
    WS_DEBUG_PRINT("[sleep] Timer wakeup set for ");
    WS_DEBUG_PRINT(msg->config.timer.duration);
    WS_DEBUG_PRINTLN(" seconds");
    break;
  case ws_sleep_Enter_ext0_tag:
    rc = _sleep_hardware->RegisterExt0Wakeup(
        msg->config.ext0.name, msg->config.ext0.level, msg->config.ext0.pull);
    if (!rc) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to set ext0 wakeup");
    }
    // TODO: Remove this debug print in production build
    WS_DEBUG_PRINT("[sleep] EXT0 wakeup set on pin ");
    WS_DEBUG_PRINT(msg->config.ext0.name);
    WS_DEBUG_PRINT(" with level ");
    WS_DEBUG_PRINT(msg->config.ext0.level ? "HIGH" : "LOW");
    WS_DEBUG_PRINTLN("");
    break;
  default:
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unknown sleep config type");
    break;
  }

  return rc;
}

/*!
    @brief  Returns the calculated sleep duration.
    @return The sleep duration in seconds.
*/
int SleepController::GetSleepDuration() {
  return _sleep_hardware->GetSleepDuration();
}

/*!
    @brief  Gets the ESP wake cause enum from hardware.
    @return The ESP wake cause enum.
*/
ws_sleep_EspWakeCause SleepController::GetEspWakeCause() {
  return _sleep_hardware->GetEspWakeCauseEnum();
}

/*!
    @brief  Returns whether the device woke from a sleep mode.
    @return True if the device woke from sleep, False otherwise.
*/
bool SleepController::DidWakeFromSleep() {
  // If SLEEP wake cause is not unspecified, device woke from sleep mode,
  // so we assume we're locked unless overridden by boot button or Enter message
  _lock = _sleep_hardware->GetEspWakeCauseEnum() !=
          ws_sleep_EspWakeCause_ESP_UNSPECIFIED;
  return _lock;
}

/*!
    @brief  Returns whether the device is in a sleep loop (locked).
    @return True if the device is locked for sleep, False otherwise.
*/
bool SleepController::IsSleepMode() { return _lock; }

/*!
    @brief  Enters the configured sleep mode.
    @return True if the device successfully entered sleep, False otherwise.
*/
void SleepController::StartSleep() {
  // Set current time before entering sleep
  _sleep_hardware->SetSleepEnterTime();

  // Enter deep sleep
  if (_sleep_mode == ws_sleep_SleepMode_S_DEEP) {
    WS_DEBUG_PRINTLN("[sleep] Entering deep sleep mode with settings:");
    esp_deep_sleep_start();
  }

  // Attempt to enter light sleep
  WS_DEBUG_PRINTLN("[sleep] Entering light sleep mode with settings:");
  esp_err_t err = esp_light_sleep_start();
  if (err != ESP_OK) {
    WS_DEBUG_PRINT("[sleep] WARNING: Failed light sleep start: ");
    WS_DEBUG_PRINTLN(esp_err_to_name(err));
  }
}

/*!
    @brief  Handles a network FSM failure when attempting to connect to WiFi
            or Adafruit IO by putting the device into sleep mode for a fixed
            duration instead of kicking the WDT.
*/
void SleepController::HandleNetFSMFailure() {
  // NOTE: We should only get here if NET_FSM_FAILURE was triggered
  // during network connection or Adafruit IO MQTT connection.
  WS_DEBUG_PRINTLN(
      "[sleep] Handling network FSM failure, entering sleep mode"); // TODO:
                                                                    // Remove in
                                                                    // production,
                                                                    // debug
                                                                    // only!
  // Create a sleep enter message to enter sleep depending on the mode we came
  // out of
  ws_sleep_Enter sleep_enter_msg = ws_sleep_Enter_init_zero;
  sleep_enter_msg.lock = true;
  // Set sleep mode to the last known sleep
  sleep_enter_msg.mode = _sleep_mode;
  sleep_enter_msg.which_config =
      _sleep_hardware->GetEspSleepSource() == ESP_SLEEP_WAKEUP_TIMER
          ? ws_sleep_Enter_timer_tag
          : ws_sleep_Enter_ext0_tag;
  // Get the previous sleep duration from RTC mem
  sleep_enter_msg.config.timer.duration = _sleep_hardware->GetSleepDuration();
  // Configure sleep mode
  Handle_Sleep_Enter(&sleep_enter_msg);
  // Enter sleep mode
  StartSleep();
}

#endif // ARDUINO_ARCH_ESP32