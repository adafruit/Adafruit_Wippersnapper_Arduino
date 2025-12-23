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
#include "nanopb/ws_pb_helpers.h"

RTC_SLOW_ATTR static struct timeval sleep_enter_time;

/*!
    @brief  Sleep controller constructor
*/
SleepController::SleepController() {
  _sleep_hardware = new SleepHardware();
  _sleep_model = new SleepModel();
  _btn_cfg_mode = false;
  CheckBootButton();
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
  WS_DEBUG_PRINTLN("[sleep] Handle_Sleep_Enter MESSAGE...");
  // TODO: Implement sleep enter logic
  return true;
}

/*!
    @brief  Reads the state of the BOOT button and stores it. Must be called
   upon class init.
*/
void SleepController::CheckBootButton() {
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  _btn_cfg_mode = (digitalRead(BOOT_BUTTON) == LOW);
  // Blink to signal we're not going to sleep again
  statusLEDBlink(WS_LED_STATUS_ERROR_RUNTIME, 3);
}

/*!
    @brief  Calculates the duration of the last sleep period.
    @returns True if the duration was successfully calculated, False otherwise.
*/
bool SleepController::CalculateSleepDuration() {
/**
 * Prefer to use RTC mem instead of NVS to save the deep sleep enter time,
 * unless the chip does not support RTC mem(such as esp32c2). Because the time
 * overhead of NVS will cause the recorded deep sleep enter time to be not very
 * accurate.
 */
#if !SOC_RTC_FAST_MEM_SUPPORTED
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  nvs_handle_t nvs_handle;
  err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR while opening NVS handle");
    return false;
  }
  // Get deep sleep enter time
  nvs_get_i32(nvs_handle, "slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
  nvs_get_i32(nvs_handle, "slp_enter_usec",
              (int32_t *)&sleep_enter_time.tv_usec);
#endif

  struct timeval now;
  gettimeofday(&now, NULL);
  _sleep_time = (now.tv_sec - sleep_enter_time.tv_sec) +
                (now.tv_usec - sleep_enter_time.tv_usec) / 1000000;
  return true;
}

/*!
    @brief  Determines the cause of the last wakeup from sleep.
    @returns True if the wakeup cause was successfully determined, False
   otherwise.
*/
bool SleepController::GetWakeupCause() {
  _wake_cause = esp_sleep_get_wakeup_cause();

  if (_wake_cause == ESP_SLEEP_WAKEUP_TIMER) {
    // TODO: From
    // https://github.com/espressif/esp-idf/blob/v5.5.1/examples/system/deep_sleep/main/deep_sleep_example_main.c,
    // audit this for clarity Calculate time spent in sleep
    if (!CalculateSleepDuration()) {
      WS_DEBUG_PRINTLN("[sleep] ERROR: Unable to get sleep duration");
      return false;
    }
    WS_DEBUG_PRINT("Time spent in sleep: ");
    WS_DEBUG_PRINTLN(_sleep_time);
  } else if (_wake_cause == ESP_SLEEP_WAKEUP_EXT0) {
    WS_DEBUG_PRINTLN("Wakeup caused by external signal using RTC_IO");
  } else if (_wake_cause == ESP_SLEEP_WAKEUP_GPIO) {
    WS_DEBUG_PRINTLN(
        "(Light Sleep) Wakeup caused by external signal using RTC_IO");
  } else {
    WS_DEBUG_PRINT("Unknown ESP_SLEEP Wake Cause: ");
    WS_DEBUG_PRINTLN(_wake_cause);
  }
  return true;
}

#endif // ARDUINO_ARCH_ESP32