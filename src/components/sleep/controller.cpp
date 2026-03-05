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
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2350)
#include "controller.h"

/*!
    @brief  Sleep controller constructor
*/
SleepController::SleepController() {
  _sleep_hardware = new SleepHardware();
  _sleep_model = new SleepModel();
  _wake_enable_pin = 255;    // No pin assigned
  _wake_enable_pin_pull = 0; // Default: no pull
  _sleep_enabled = false;    // Initially we are not in sleep mode

// Mark so we can disable all external peripherals that draw power during sleep
// (i.e: tft, i2c, neopixel, etc)
#if defined(NEOPIXEL_POWER) || defined(PIN_I2C_POWER) || defined(TFT_POWER) || \
    defined(TFT_I2C_POWER) || defined(NEOPIXEL_I2C_POWER)
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
  case ws_sleep_B2D_sleep_config_tag:
    // Lock is always true when receiving a standalone sleep config command
    res = handleSleepConfig(&b2d.payload.sleep_config, true);
    break;
  default:
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unsupported Sleep payload");
    break;
  }

  return res;
}

/*!
    @brief  Handles a Sleep Enter message from the broker.
    @param  msg
            The Sleep Enter message.
    @param  enable
            Whether to enable sleep mode. When true, the device will enter
            sleep. When false, the device resumes regular operation.
    @return True if the sleep mode was successfully entered, False otherwise.
*/
bool SleepController::handleSleepConfig(ws_sleep_SleepConfig *msg,
                                        bool enable) {
  WS_DEBUG_PRINTLN("[sleep] handleSleepConfig()");

  // Parse and handle sleep state
  _sleep_enabled = enable;

  // If wake enable pin is "active", override sleep enable
  _wake_enable_pin_state = CheckWakeEnablePin();
  if (_wake_enable_pin_state && _sleep_enabled) {
    WS_DEBUG_PRINTLN(
        "[sleep] Wake enable pin is active, overriding sleep enable.");
    _sleep_enabled = false;
    return true;
  }

  if (!_sleep_enabled) {
    WS_DEBUG_PRINTLN("[sleep] Sleep disabled, resuming regular operation.");
    return true;
  }

  // Parse sleep mode and store to RTC memory for persistence across sleep
  _sleep_hardware->SetSleepMode(msg->mode);

  // Configure sleep with the provided wakeup configuration
  bool res = false;
  ws_sleep_SleepMode sleep_mode = _sleep_hardware->GetSleepMode();
  if (sleep_mode == ws_sleep_SleepMode_S_DEEP ||
      sleep_mode == ws_sleep_SleepMode_S_LIGHT) {
    res = ConfigureSleep(msg);
  } else {
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unsupported sleep mode");
  }

  WS_DEBUG_PRINTLN("[sleep] Sleep configuration complete.");
  WS_DEBUG_PRINT("Sleep Enabled: ");
  WS_DEBUG_PRINTLN(_sleep_enabled ? "YES" : "NO");

  return res;
}

/*!
    @brief  Handles wake-up from light sleep mode. Refreshes the cached
            wakeup cause and re-registers the RTC timer if applicable.
*/
void SleepController::WakeFromLightSleep() {
  Serial.begin(115200);
  // If already enumerated, additional class driver begin() e.g msc, hid, midi
  // won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Refresh the cached sleep wakeup cause from hardware
  _sleep_hardware->GetSleepWakeupCause();
  // Verify that we woke up from light sleep
  if (!DidWakeFromSleep() || (GetPrvSleepMode() != ws_sleep_SleepMode_S_LIGHT))
    return;
  WS_DEBUG_PRINTLN("[sleep] Woke up from light sleep!");

  // Recalculate sleep duration
  _sleep_hardware->CalculateSleepDuration();
  WS_DEBUG_PRINT("Slept for ");
  WS_DEBUG_PRINT(GetSleepDurationSecs());
  WS_DEBUG_PRINTLN(" seconds");

  // Print sleep cycles
  WS_DEBUG_PRINT("Total light sleep cycles: ");
  WS_DEBUG_PRINTLN(_sleep_hardware->GetSleepCycleCount());

  // Re-enable external components that were disabled before sleep
  if (_has_ext_pwr_components) {
    _sleep_hardware->ReenableExternalComponents();
    WS_DEBUG_PRINTLN("[sleep] Re-enabled external components");
  }

  // Re-initialize SD card if it was previously initialized
  if (Ws._sdCardV2->isModeOffline()) {
    if (!Ws._sdCardV2->begin()) {
      Ws.haltErrorV2(
          "[sleep] ERROR: Failed to re-initialize SD card after wake");
    }
    WS_DEBUG_PRINTLN("[sleep] SD card re-initialized successfully");
  }

#ifdef USE_STATUS_LED
  // Visual indication for configuring sleep in RP2350
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
#endif
}

/*!
    @brief  Configures the sleep mode based on the provided Enter message.
    @param  msg
            Pointer to the Sleep Enter message containing config union.
    @return True if deep sleep was successfully configured, False otherwise.
*/
bool SleepController::ConfigureSleep(const ws_sleep_SleepConfig *msg) {
  bool rc = false;
  switch (msg->which_config) {
  case ws_sleep_SleepConfig_timer_tag:
// Configure timer-based wakeup source
#ifdef ARDUINO_ARCH_ESP32
    rc = _sleep_hardware->RegisterRTCTimerWakeup(msg->config.timer.duration *
                                                 1000000);
    if (!rc) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to set timer wakeup");
    }
#else
    Ws._wdt->registerTimerWakeup(msg->config.timer.duration * 1000);
    rc = true;
#endif

    WS_DEBUG_PRINT("[sleep] Timer wakeup set to ");
    WS_DEBUG_PRINT(msg->config.timer.duration);
    WS_DEBUG_PRINTLN(" seconds");
    break;
  case ws_sleep_SleepConfig_ext0_tag:
#ifdef ARDUINO_ARCH_ESP32
    rc = _sleep_hardware->RegisterExt0Wakeup(msg->config.ext0.pin_name,
                                             msg->config.ext0.level,
                                             msg->config.ext0.pull);
    if (!rc) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to set ext0 wakeup");
    }
#else
  {
    // Convert pin name string to numeric GPIO pin
    const char *pin_num = msg->config.ext0.pin_name;
    if ((pin_num[0] == 'D' || pin_num[0] == 'A') && pin_num[1] != '\0') {
      pin_num = pin_num + 1;
    }
    uint gpio_pin = (uint)strtol(pin_num, nullptr, 10);
    Ws._wdt->registerGPIOWakeup(gpio_pin, true, msg->config.ext0.level);
    rc = true;
  }
#endif
    WS_DEBUG_PRINT("[sleep] EXT0 wakeup set on pin ");
    WS_DEBUG_PRINT(msg->config.ext0.pin_name);
    WS_DEBUG_PRINT(" | Level ");
    WS_DEBUG_PRINT(msg->config.ext0.level ? "HIGH" : "LOW");
    WS_DEBUG_PRINT(" | Pull: ");
    WS_DEBUG_PRINTLN(msg->config.ext0.pull ? "ENABLED" : "DISABLED");
    break;
  default:
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unknown sleep config type");
    break;
  }

#ifdef USE_STATUS_LED
  // Visual indication for configuring sleep in RP2350
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
#endif

  return rc;
}

/*!
    @brief  Sets the wake enable pin to check before entering sleep.
    @param  pin
            The GPIO pin number.
    @param  pull
            Pull mode (0 = none, 1 = pull-down, 2 = pull-up)
*/
void SleepController::SetWakeEnablePin(uint8_t pin, uint8_t pull) {
  _wake_enable_pin = pin;
  _wake_enable_pin_pull = pull;
}

/*!
    @brief  Checks the wake enable pin state before entering sleep.
    @return True if wake enable pin is active (stay awake), False otherwise.
    @note   For pull-up: active = LOW (button pressed to GND)
            For pull-down/no pull: active = HIGH (button pressed to VCC)
*/
bool SleepController::CheckWakeEnablePin() {
  // Configure pin with specified pull mode
  switch (_wake_enable_pin_pull) {
  case 1:
    pinMode(_wake_enable_pin, INPUT_PULLDOWN);
    break;
  case 2:
    pinMode(_wake_enable_pin, INPUT_PULLUP);
    break;
  default:
    pinMode(_wake_enable_pin, INPUT);
    break;
  }
  int pinState = digitalRead(_wake_enable_pin);

  // Return whether pin state indicates "active" based on pull mode
  if (_wake_enable_pin_pull == 2) {
    // Pull-up is an active LOW
    return (pinState == LOW);
  }
  // Otherwise, pull-down/no pull is an active HIGH
  return (pinState == HIGH);
}

/*!
    @brief  Returns the calculated sleep duration.
    @return The sleep duration in seconds.
*/
int SleepController::GetSleepDurationSecs() {
  return _sleep_hardware->GetSleepDurationSecs();
}

/*!
    @brief  Returns the run duration before sleep entry, in milliseconds.
    @return The run duration in milliseconds.
*/
unsigned long SleepController::getRunDurationMs() {
  return (unsigned long)_sleep_model->getRunDurationMs() * 1000UL;
}

/*!
    @brief  Gets the wakeup reason as a human-readable string.
    @return C string describing the wakeup reason.
*/
const char *SleepController::GetWakeupReasonName() {
  return _sleep_hardware->GetWakeupReasonName();
}

/*!
    @brief  Returns whether the device woke from sleep mode.
    @return True if the device woke from sleep, False otherwise.
*/
bool SleepController::DidWakeFromSleep() {
  bool woke_from_sleep = false;
#ifdef ARDUINO_ARCH_ESP32
  // If sleep source is not undefined, device woke from sleep mode,
  // so we assume we're locked unless overridden by boot button or Enter message
  esp_sleep_source_t wake_source = _sleep_hardware->GetEspSleepSource();
  woke_from_sleep = wake_source != ESP_SLEEP_WAKEUP_UNDEFINED;
#else
  // RP2350 doesn't track wake cause but does internally track if we slept or
  // not
  woke_from_sleep = Ws._wdt->didWakeFromSleep();
#endif
  return woke_from_sleep;
}

/*!
    @brief  Returns the previous sleep mode before wakeup.
    @return The previous sleep cycle's mode as ws_sleep_SleepMode enum.
*/
ws_sleep_SleepMode SleepController::GetPrvSleepMode() {
  return _sleep_hardware->GetSleepMode();
}

/*!
    @brief  Returns if sleep mode is active for the current boot cycle.
    @return True if sleep mode is active, False otherwise.
*/
bool SleepController::isSleepEnabled() { return _sleep_enabled; }

/*!
    @brief  Enters the configured sleep mode.
*/
void SleepController::StartSleep() {
  // DEBUG: D13 OFF before sleep
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Set current time before entering sleep
  _sleep_hardware->SetSleepEnterTime();

  // Store soft RTC counter to RTC memory before sleep
  if (Ws._sdCardV2 != nullptr && Ws._sdCardV2->isRTCSoft()) {
    uint32_t counter = Ws._sdCardV2->GetSoftRTCTime();
    SetSoftRtcCounter(counter);
    WS_DEBUG_PRINT("[sleep] Stored soft RTC counter: ");
    WS_DEBUG_PRINTLN(counter);
  }

  // Disable any external components that draw power during sleep
  if (_has_ext_pwr_components) {
    WS_DEBUG_PRINTLN(
        "[sleep] Disabling externally powered components before sleep...");
    _sleep_hardware->DisableExternalComponents();
  }

  // Disable SD Card, if present
  if (Ws._sdCardV2 != nullptr && Ws._sdCardV2->isSDCardInitialized()) {
    WS_DEBUG_PRINTLN("[sleep] Disabling SD card before sleep...");
    Ws._sdCardV2->end();
  }

  ws_sleep_SleepMode sleep_mode = _sleep_hardware->GetSleepMode();
  if (sleep_mode == ws_sleep_SleepMode_S_DEEP) {
    WS_DEBUG_PRINTLN("[sleep] Entering deep sleep");
#ifdef ARDUINO_ARCH_ESP32
    esp_deep_sleep_start();
#else
    WS_DEBUG_PRINTLN("[sleep] ERROR: RP2350 does not support deep sleep mode, "
                     "cannot enter deep sleep.");
#endif
  } else if (sleep_mode == ws_sleep_SleepMode_S_LIGHT) {
    WS_DEBUG_PRINTLN("[sleep] Entering light sleep");
// Attempt to start light sleep
#ifdef ARDUINO_ARCH_ESP32
    // Flush serial output before entering light sleep
    WS_DEBUG_PRINTLN("[sleep] Starting light sleep NOW...");
    Serial.flush();
    Serial.end();
    // Start light sleep
    esp_err_t err = esp_light_sleep_start();
    // === WOKE UP FROM LIGHT SLEEP ===
    WS_DEBUG_PRINTLN("[sleep] *** WOKE FROM LIGHT SLEEP ***");
    WS_PRINTER.flush();
    if (err != ESP_OK) {
      WS_DEBUG_PRINT("[sleep] WARNING: Failed light sleep start: ");
      WS_DEBUG_PRINTLN(esp_err_to_name(err));
    }
#else
// Start light sleep using sleepydog for RP2350
#ifdef USE_STATUS_LED
    digitalWrite(STATUS_LED_PIN, LOW);
#endif
    Ws._wdt->startSleep();
#endif
  } else {
    WS_DEBUG_PRINTLN(
        "[sleep] WARNING: Unsupported sleep mode, not entering sleep.");
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
  ws_sleep_SleepConfig msg_sleep_cfg = ws_sleep_SleepConfig_init_zero;
  // Set sleep mode to the last known sleep
  msg_sleep_cfg.mode = _sleep_hardware->GetSleepMode();
#ifdef ARDUINO_ARCH_ESP32
  msg_sleep_cfg.which_config =
      _sleep_hardware->GetEspSleepSource() == ESP_SLEEP_WAKEUP_TIMER
          ? ws_sleep_SleepConfig_timer_tag
          : ws_sleep_SleepConfig_ext0_tag;
#else
  msg_sleep_cfg.which_config = Ws._wdt->isSleepConfigTimer()
                                   ? ws_sleep_SleepConfig_timer_tag
                                   : ws_sleep_SleepConfig_ext0_tag;
#endif
  // Get the previous sleep duration from RTC mem
  msg_sleep_cfg.config.timer.duration = _sleep_hardware->GetSleepDurationSecs();
  // Configure sleep mode (sleep_enabled=true to re-enter sleep after FSM
  // failure)
  handleSleepConfig(&msg_sleep_cfg, true);
  // Enter sleep mode
  StartSleep();
}

/*!
    @brief  Gets the stored software RTC counter from RTC memory.
    @return The stored software RTC counter.
*/
uint32_t SleepController::GetSoftRtcCounter() {
  return _sleep_hardware->GetPrvSoftRtcCounter();
}

/*!
    @brief  Sets the software RTC counter in RTC memory for persistence
            across sleep cycles.
    @param  counter
            The counter value to store.
*/
void SleepController::SetSoftRtcCounter(uint32_t counter) {
  _sleep_hardware->StoreSoftRtcCounter(counter);
}

/*!
    @brief  Sets the log filename in RTC memory for persistence across sleep.
    @param  filename
            The log filename to store.
*/
void SleepController::SetLogFilename(const char *filename) {
  _sleep_hardware->StoreLogFilename(filename);
}

/*!
    @brief  Gets the stored log filename from NVS.
    @return The stored log filename.
*/
const char *SleepController::GetLogFilename() {
  return _sleep_hardware->GetLogFilename();
}

#endif // ARDUINO_ARCH_ESP32 || ARDUINO_ARCH_RP2350