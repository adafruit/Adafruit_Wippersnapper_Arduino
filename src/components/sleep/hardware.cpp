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
RTC_DATA_ATTR ws_sleep_SleepMode sleep_mode;

/*!
    @brief  Sleep hardware constructor
*/
SleepHardware::SleepHardware() {
  GetSleepWakeupCause();
  CalculateSleepDuration();
}

/*!
    @brief  Sleep hardware destructor
*/
SleepHardware::~SleepHardware() {}

/*!
    @brief  Gets the wakeup source which caused wakeup from sleep.
*/
void SleepHardware::GetSleepWakeupCause() {
  _wakeup_cause = esp_sleep_get_wakeup_cause();
}

/*!
    @brief  Returns the esp_sleep_source_t wakeup cause.
    @return The wakeup cause as esp_sleep_source_t.
*/
esp_sleep_source_t SleepHardware::GetEspSleepSource() { return _wakeup_cause; }

/*!
    @brief  Returns the ESP wake cause enum corresponding to the internal
            wakeup cause.
    @return The wake cause as a ws_sleep_EspWakeCause enum.
*/
ws_sleep_EspWakeCause SleepHardware::GetEspWakeCauseEnum() {
  switch (_wakeup_cause) {
  case ESP_SLEEP_WAKEUP_ALL:
    return ws_sleep_EspWakeCause_ESP_ALL;
  case ESP_SLEEP_WAKEUP_EXT0:
    return ws_sleep_EspWakeCause_ESP_EXT0;
  case ESP_SLEEP_WAKEUP_EXT1:
    return ws_sleep_EspWakeCause_ESP_EXT1;
  case ESP_SLEEP_WAKEUP_TIMER:
    return ws_sleep_EspWakeCause_ESP_TIMER;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    return ws_sleep_EspWakeCause_ESP_TOUCHPAD;
  case ESP_SLEEP_WAKEUP_ULP:
    return ws_sleep_EspWakeCause_ESP_ULP;
  case ESP_SLEEP_WAKEUP_GPIO:
    return ws_sleep_EspWakeCause_ESP_GPIO;
  case ESP_SLEEP_WAKEUP_UART:
    return ws_sleep_EspWakeCause_ESP_UART;
  case ESP_SLEEP_WAKEUP_WIFI:
    return ws_sleep_EspWakeCause_ESP_WIFI;
  case ESP_SLEEP_WAKEUP_COCPU:
    return ws_sleep_EspWakeCause_ESP_COCPU;
  case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG:
    return ws_sleep_EspWakeCause_ESP_COCPU_TRAP;
  case ESP_SLEEP_WAKEUP_BT:
    return ws_sleep_EspWakeCause_ESP_BT;
  case ESP_SLEEP_WAKEUP_VAD:
    return ws_sleep_EspWakeCause_ESP_VAD;
  case ESP_SLEEP_WAKEUP_VBAT_UNDER_VOLT:
    return ws_sleep_EspWakeCause_ESP_VBAT_UNDER_VOLT;
  default:
    return ws_sleep_EspWakeCause_ESP_UNSPECIFIED;
  }
}

/*!
    @brief  Returns the current (or previously entered) sleep mode. This mode
            is set when enabling deep or light sleep, and can be retrieved
            post-sleep from the RTC memory.
    @return The sleep mode.
*/
ws_sleep_SleepMode SleepHardware::GetSleepMode() { return sleep_mode; }

/*!
    @brief  Calculates the duration of the last sleep period.
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
    @brief  Returns the calculated sleep duration.
    @return The sleep duration in seconds.
*/
int SleepHardware::GetSleepDuration() { return _sleep_time; }

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

  sleep_mode = ws_sleep_SleepMode_S_DEEP;
  return true;
}

// TODO: Not implemented yet!
bool SleepHardware::EnableLightSleep(int duration) {
  sleep_mode = ws_sleep_SleepMode_S_LIGHT;
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

/*!
    @brief  Disables power to all external components that may draw power
            during sleep (i.e: tft, i2c, neopixel, etc)
*/
void SleepHardware::DisableExternalComponents() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(TFT_POWER)
  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, LOW);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // (specific to adafruit feather esp32-s2) - turn off the I2C power by setting
  // pin to rest state (off)
  pinMode(PIN_I2C_POWER, INPUT);
#elif defined(PIN_I2C_POWER)
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, LOW);
#elif defined(TFT_I2C_POWER)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, LOW);
#endif
}

#endif // ARDUINO_ARCH_ESP32