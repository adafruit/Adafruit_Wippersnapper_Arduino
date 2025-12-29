/*!
 * @file src/components/sleep/hardware.cpp
 *
 * Hardware implementation for the sleep.proto message.
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
#include "hardware.h"
#include <stdlib.h>
RTC_SLOW_ATTR static struct timeval sleep_enter_time;

/*!
    @brief  Sleep hardware constructor
*/
SleepHardware::SleepHardware() {
  _wakeup_cause = ESP_SLEEP_WAKEUP_UNDEFINED;
  GetWakeupCause();
}

/*!
    @brief  Sleep hardware destructor
*/
SleepHardware::~SleepHardware() {}

/*!
    @brief  Determines the cause of the last wakeup from sleep.
    @returns True if the wakeup cause was successfully determined, False
   otherwise.
*/
void SleepHardware::GetWakeupCause() {
  _wakeup_cause = esp_sleep_get_wakeup_cause();
  if (_wakeup_cause == ESP_SLEEP_WAKEUP_TIMER) {
    CalculateSleepDuration();
  }
}

/*!
    @brief  Calculates the duration of the last sleep period.
    @returns True if the duration was successfully calculated, False otherwise.
*/
void SleepHardware::CalculateSleepDuration() {
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
    return;
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
}

/*!
    @brief   Enables deep sleep mode for a specified duration.
    @param   duration Time to deep sleep, in seconds.
    @returns True if deep sleep was successfully enabled, False otherwise.
*/
bool SleepHardware::EnableDeepSleep(int duration) {
  esp_err_t rc = esp_sleep_enable_timer_wakeup(duration * 1000000);
  if (rc != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to enable timer wakeup");
    return false;
  }

#if CONFIG_IDF_TARGET_ESP32
  // Isolate GPIO12 pin from external circuits. This is needed for modules
  // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
  // to minimize current consumption.
  rtc_gpio_isolate(GPIO_NUM_12);
#endif

  return true;
}

/*!
    @brief  Reads the state of the BOOT button and stores it. Must be called
   upon class init.
*/
bool SleepHardware::CheckBootButton() {
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  bool res = (digitalRead(BOOT_BUTTON) == LOW);
  // Blink to signal we're not going to sleep again
  statusLEDBlink(WS_LED_STATUS_ERROR_RUNTIME, 3);
  return res;
}

/*!
    @brief Enables wakeup by timer.
    @param duration
           Time before wakeup, in microseconds
*/
bool SleepHardware::RegisterRTCTimerWakeup(uint64_t duration) {
  esp_err_t rc = esp_sleep_enable_timer_wakeup(duration);
  if (rc != ESP_OK)
    return false;
  return true;
}

bool SleepHardware::RegisterExt0Wakeup(const char *pin_name, bool pin_level,
                                       bool pin_pull) {
  esp_err_t rc;
  if (!pin_name) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: pin_name is null");
    return false;
  }

  const char *numeric = pin_name;
  if ((pin_name[0] == 'D' || pin_name[0] == 'A') && pin_name[1] != '\0') {
    numeric = pin_name + 1;
  }

  char *endptr = nullptr;
  long pin_num = strtol(numeric, &endptr, 10);
  if (endptr == numeric || pin_num < 0) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Invalid pin name for ext0 wakeup");
    return false;
  }
  gpio_num_t pin = (gpio_num_t)pin_num;

  // Configure pull resistor, if enabled
  if (pin_pull) {
    if (pin_level) {
      // Waking on HIGH, pull DOWN to keep inactive
      rtc_gpio_pullup_dis(pin);
      rtc_gpio_pulldown_en(pin);
    } else {
      // Waking on LOW, pull UP to keep inactive
      rtc_gpio_pulldown_dis(pin);
      rtc_gpio_pullup_en(pin);
    }
  } else {
    // No pull resistors enabled
    rtc_gpio_pullup_dis(pin);
    rtc_gpio_pulldown_dis(pin);
  }

  rc = esp_sleep_enable_ext0_wakeup(pin, pin_level ? 1 : 0);
  if (rc != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to enable ext0 wakeup");
    return false;
  }

  return true;
}

#endif // ARDUINO_ARCH_ESP32