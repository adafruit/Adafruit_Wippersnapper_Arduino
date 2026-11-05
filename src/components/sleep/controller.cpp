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
#include "components/digital/hardware.h"

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
  WS_DEBUG_PRINTLNVAR(_sleep_enabled ? "YES" : "NO");

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
  // TODO: This doesn't compile on ESP32-C3, do we need it?
  /*   if (TinyUSBDevice.mounted()) {
      TinyUSBDevice.detach();
      delay(10);
      TinyUSBDevice.attach();
    } */

  // Refresh the cached sleep wakeup cause from hardware
  _sleep_hardware->GetSleepWakeupCause();
  // Verify that we woke up from light sleep
  if (!DidWakeFromSleep() || (GetPrvSleepMode() != ws_sleep_SleepMode_S_LIGHT))
    return;
  WS_DEBUG_PRINTLN("[sleep] Woke up from light sleep!");

#if defined(ARDUINO_ARCH_ESP32) &&                                             \
    !(defined(CONFIG_IDF_TARGET_ESP32C3) ||                                    \
      defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6))
  // ext0 leaves the wake pin's pad routed to RTC IO after waking. Light sleep
  // doesn't reboot, so nothing re-runs pinMode() - hand the pad back to the
  // digital GPIO matrix so the Digital component owning it can read it again.
  // Analog pins are left alone, the ADC reads through the RTC/analog pad.
  const char *ext0_pin_name = _sleep_hardware->GetEspSleepExt0PinName();
  if (ext0_pin_name != nullptr) {
    int32_t ext0_pin = WsPinNameToNum(ext0_pin_name);
    if (ext0_pin != WS_PIN_INVALID &&
        Ws->digital_controller->getPinPullDirection((uint8_t)ext0_pin,
                                                    nullptr)) {
      rtc_gpio_deinit((gpio_num_t)ext0_pin);
      WS_DEBUG_PRINTLN("[sleep] Released ext0 wake pin back to digital GPIO");
    }
  }
#endif

  // Recalculate sleep duration
  _sleep_hardware->CalculateSleepDuration();
  WS_DEBUG_PRINT("Slept for ");
  WS_DEBUG_PRINTVAR(GetSleepDurationSecs());
  WS_DEBUG_PRINTLN(" seconds");

  // Print sleep cycles
  WS_DEBUG_PRINT("Total light sleep cycles: ");
  WS_DEBUG_PRINTLNVAR(_sleep_hardware->GetSleepCycleCount());

  // Re-enable external components that were disabled before sleep
  if (_has_ext_pwr_components) {
    _sleep_hardware->ReenableExternalComponents();
    WS_DEBUG_PRINTLN("[sleep] Re-enabled external components");
  }

  // Re-initialize SD card if it was previously initialized
  if (Ws->_sdCardV2->isModeOffline()) {
    if (!Ws->_sdCardV2->begin()) {
      Ws->haltErrorV2(
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
    @brief  Checks that an ext0 wakeup pin is already configured by the
            Digital or Analog controller and retrieves its pull direction.
            Publishes a component error if the pin can not be used.
    @param  pin_name
            The ext0 wakeup pin's name (e.g. "D5", "A0").
    @param  pull_direction
            Out: the pin's pull direction (D_INPUT_PULL_UP,
            D_INPUT_PULL_DOWN, or D_INPUT for no pull).
    @return True if the pin is configured by a component, False otherwise.
*/
bool SleepController::getExt0PinPullDirection(
    const char *pin_name, ws_digital_Direction *pull_direction) {
  uint8_t pin_num = 0;
  ExpanderHardware *expander_drv = nullptr;
  if (!Ws->_expander_controller->ResolvePinName(pin_name, pin_num,
                                                &expander_drv)) {
    Ws->error_handler->publishComponentError(pin_name,
                                             "Unable to resolve pin name");
    return false;
  }

  // ext0 can only wake from a native GPIO pin
  if (expander_drv != nullptr) {
    Ws->error_handler->publishComponentError(pin_name,
                                             "Wake pin must be a native GPIO");
    return false;
  }

#if defined(ARDUINO_ARCH_ESP32) &&                                             \
    !(defined(CONFIG_IDF_TARGET_ESP32C3) ||                                    \
      defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6))
  // ext0 wakeup only works on RTC-capable GPIO pins
  if (!rtc_gpio_is_valid_gpio((gpio_num_t)pin_num)) {
    Ws->error_handler->publishComponentError(
        pin_name, "Wake pin is not an RTC GPIO, can not be used for ext0");
    return false;
  }
#endif

  // The pin must already be configured by the Digital or Analog controller
  if (Ws->digital_controller->getPinPullDirection(pin_num, pull_direction)) {
    return true;
  }
  if (Ws->analog_controller->getPinPullDirection(pin_num, pull_direction)) {
    return true;
  }

  Ws->error_handler->publishComponentError(
      pin_name, "Wake pin is not configured as a digital or analog component");
  return false;
}

/*!
    @brief  Configures the sleep mode based on the provided Enter message.
    @param  msg
            Pointer to the Sleep Enter message containing config union.
    @return True if deep sleep was successfully configured, False otherwise.
*/
bool SleepController::ConfigureSleep(const ws_sleep_SleepConfig *msg) {
  bool rc = false;
#ifdef ARDUINO_ARCH_ESP32
  // Wakeup sources are NOT disabled automatically after waking, clear any
  // source left armed by a previous sleep cycle so only this config applies.
  // ESP_ERR_INVALID_STATE (nothing armed yet) is expected and ignored.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#endif
  if (msg->has_timer) {
// Configure timer-based wakeup source
#ifdef ARDUINO_ARCH_ESP32
    rc = _sleep_hardware->RegisterRTCTimerWakeup(
        (uint64_t)msg->timer.duration * ONE_SECOND_IN_US);
    if (!rc) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to set timer wakeup");
    }
#else
    Ws->_wdt->registerTimerWakeup(msg->timer.duration * 1000);
    rc = true;
#endif

    WS_DEBUG_PRINT("[sleep] Timer wakeup set to ");
    WS_DEBUG_PRINTVAR(msg->timer.duration);
    WS_DEBUG_PRINTLN(" seconds");
  } else if (_sleep_model->HasExt0()) {
    const ws_sleep_Ext0Config *ext0 = _sleep_model->GetExt0Config();

    // The wake pin must already be configured by the Digital or Analog
    // controller, its pull direction is taken from that component
    ws_digital_Direction pull_direction = ws_digital_Direction_D_INPUT;
    if (!getExt0PinPullDirection(ext0->pin, &pull_direction)) {
      return false;
    }

#ifdef ARDUINO_ARCH_ESP32
    rc = _sleep_hardware->RegisterExt0Wakeup(ext0->pin, ext0->level,
                                             pull_direction);
    if (!rc) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to set ext0 wakeup");
    }
#else
    {
      // Resolve the pin name the same way the Digital/Analog controllers do,
      // already validated by getExt0PinPullDirection()
      uint gpio_pin = (uint)WsPinNameToNum(ext0->pin);
      Ws->_wdt->registerGPIOWakeup(gpio_pin, true, ext0->level);
      rc = true;
    }
#endif
    WS_DEBUG_PRINT("[sleep] EXT0 wakeup set on pin ");
    WS_DEBUG_PRINTVAR(ext0->pin);
    WS_DEBUG_PRINT(" | Level ");
    if (ext0->level) {
      WS_DEBUG_PRINTVAR("HIGH");
    } else {
      WS_DEBUG_PRINTVAR("LOW");
    }
    WS_DEBUG_PRINT(" | Pull: ");
    if (pull_direction == ws_digital_Direction_D_INPUT_PULL_UP) {
      WS_DEBUG_PRINTLNVAR("UP");
    } else if (pull_direction == ws_digital_Direction_D_INPUT_PULL_DOWN) {
      WS_DEBUG_PRINTLNVAR("DOWN");
    } else {
      WS_DEBUG_PRINTLNVAR("NONE");
    }
  } else {
    WS_DEBUG_PRINTLN("[sleep] WARNING: Unknown sleep config type");
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
  // Falls back to a floating input if the core can't set the requested pull
  uint8_t mode = INPUT;
  if (_wake_enable_pin_pull == 1 || _wake_enable_pin_pull == 2) {
    ws_digital_Direction pull = (_wake_enable_pin_pull == 1)
                                    ? ws_digital_Direction_D_INPUT_PULL_DOWN
                                    : ws_digital_Direction_D_INPUT_PULL_UP;
    if (!DigitalHardware::GetInputMode(pull, &mode)) {
      WS_DEBUG_PRINTLN("[sleep] WARNING: Wake enable pin pull not supported "
                       "on this platform, using INPUT");
    }
  }
  pinMode(_wake_enable_pin, mode);
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
  woke_from_sleep = Ws->_wdt->didWakeFromSleep();
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
    @brief  Publishes a Goodnight message to the broker.
    @return True if the message was successfully published, False otherwise.
*/
bool SleepController::publishMsgGoodnight() {

  ws_sleep_D2B goodnight_d2b = ws_sleep_D2B_init_zero;
  goodnight_d2b.which_payload = ws_sleep_D2B_goodnight_tag;
  _sleep_model->EncodeSleepGoodnight(nullptr);
  goodnight_d2b.payload.goodnight = *_sleep_model->GetSleepGoodnight();

  if (!Ws->PublishD2b(ws_signal_DeviceToBroker_sleep_tag, &goodnight_d2b))
    return false;
  return true;
}

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
  if (Ws->_sdCardV2 != nullptr && Ws->_sdCardV2->isRTCSoft()) {
    uint32_t counter = Ws->_sdCardV2->GetSoftRTCTime();
    SetSoftRtcCounter(counter);
    WS_DEBUG_PRINT("[sleep] Stored soft RTC counter: ");
    WS_DEBUG_PRINTLNVAR(counter);
  }

  // Disable any external components that draw power during sleep
  if (_has_ext_pwr_components) {
    WS_DEBUG_PRINTLN(
        "[sleep] Disabling externally powered components before sleep...");
    _sleep_hardware->DisableExternalComponents();
  }

  // Disable SD Card, if present
  if (Ws->_sdCardV2 != nullptr && Ws->_sdCardV2->isSDCardInitialized()) {
    WS_DEBUG_PRINTLN("[sleep] Disabling SD card before sleep...");
    Ws->_sdCardV2->end();
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
      WS_DEBUG_PRINTLNVAR(esp_err_to_name(err));
    }
#else
// Start light sleep using sleepydog for RP2350
#ifdef USE_STATUS_LED
    digitalWrite(STATUS_LED_PIN, LOW);
#endif
    Ws->_wdt->startSleep();
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

  // Restore the last-known wakeup source. A stored ext0 pin name means sleep
  // was configured for ext0, otherwise fall back to a timer wakeup.
#ifdef ARDUINO_ARCH_ESP32
  const char *ext0_pin_name = _sleep_hardware->GetEspSleepExt0PinName();
#else
  const char *ext0_pin_name = nullptr;
#endif
  if (ext0_pin_name != nullptr) {
#ifdef ARDUINO_ARCH_ESP32
    // Re-arm the last known ext0 configuration directly from RTC memory. No
    // Digital/Analog pins are configured yet (we never checked in), so the
    // component lookup in ConfigureSleep() would fail and leave the device
    // asleep without a wakeup source.
    // Copy out of RTC memory first - RegisterExt0Wakeup() writes the pin name
    // back into the same RTC buffer, so the strncpy must not overlap
    char ext0_pin[sizeof(ws_sleep_Ext0Config::pin)];
    strncpy(ext0_pin, ext0_pin_name, sizeof(ext0_pin) - 1);
    ext0_pin[sizeof(ext0_pin) - 1] = '\0';
    _sleep_enabled = true;
    _sleep_hardware->SetSleepMode(msg_sleep_cfg.mode);
    if (!_sleep_hardware->RegisterExt0Wakeup(
            ext0_pin, _sleep_hardware->GetEspSleepExt0Level(),
            _sleep_hardware->GetEspSleepExt0PullDirection())) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to re-arm ext0 wakeup");
    }
#endif
  } else {
    // Set the timer duration to the last known sleep duration, so we can
    // re-enter sleep
    msg_sleep_cfg.has_timer = true;
    msg_sleep_cfg.timer.duration = _sleep_hardware->GetSleepDurationSecs();
    // Configure sleep mode (sleep_enabled=true to re-enter sleep after FSM
    // failure)
    handleSleepConfig(&msg_sleep_cfg, true);
  }
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