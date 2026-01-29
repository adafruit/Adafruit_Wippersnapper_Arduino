/*!
 * @file src/components/sleep/hardware.cpp
 *
 * Hardware implementation for the sleep.proto message.
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
#include "hardware.h"
#include <stdlib.h>
// Stored in RTC memory to persist across deep sleep cycles
RTC_SLOW_ATTR static struct timeval sleep_enter_time;
RTC_SLOW_ATTR static uint32_t cnt_soft_rtc;
RTC_DATA_ATTR ws_sleep_SleepMode sleep_mode;
RTC_DATA_ATTR static uint32_t sleep_cycles;

/*!
    @brief  Sleep hardware constructor
*/
SleepHardware::SleepHardware() {
  GetSleepWakeupCause();
  CalculateSleepDuration();
  cnt_soft_rtc = 0;
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
    @brief  Returns a human-readable name for the wakeup cause.
    @return C string describing the wakeup reason.
*/
const char *SleepHardware::GetWakeupReasonName() {
  switch (_wakeup_cause) {
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    return "Undefined";
  case ESP_SLEEP_WAKEUP_ALL:
    return "All";
  case ESP_SLEEP_WAKEUP_EXT0:
    return "EXT0";
  case ESP_SLEEP_WAKEUP_EXT1:
    return "EXT1";
  case ESP_SLEEP_WAKEUP_TIMER:
    return "Timer";
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    return "Touchpad";
  case ESP_SLEEP_WAKEUP_ULP:
    return "ULP";
  case ESP_SLEEP_WAKEUP_GPIO:
    return "GPIO";
  case ESP_SLEEP_WAKEUP_UART:
    return "UART";
  case ESP_SLEEP_WAKEUP_WIFI:
    return "WiFi";
  case ESP_SLEEP_WAKEUP_COCPU:
    return "COCPU";
  case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG:
    return "COCPU_Trap";
  case ESP_SLEEP_WAKEUP_BT:
    return "Bluetooth";
  default:
    return "Unknown";
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
  // Initialize NVS (required once before any NVS operations)
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // Get deep sleep enter time from NVS
  NvsReadI32("slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
  NvsReadI32("slp_enter_usec", (int32_t *)&sleep_enter_time.tv_usec);
#endif

  struct timeval now;
  gettimeofday(&now, NULL);
  _sleep_time = (now.tv_sec - sleep_enter_time.tv_sec) +
                (now.tv_usec - sleep_enter_time.tv_usec) / 1000000;

  // Update sleep cycle count
  sleep_cycles += 1;
#if !SOC_RTC_FAST_MEM_SUPPORTED
  NvsWriteU32("cnt_sleep_cycles", sleep_cycles);
#endif
}

/*!
    @brief  Returns the calculated sleep duration.
    @return The sleep duration in seconds.
*/
int SleepHardware::GetSleepDuration() { return _sleep_time; }

/*!
    @brief  Sets the sleep enter time before entering sleep mode.
            On chips with RTC memory, stores in RTC_SLOW_ATTR variable.
            On chips without RTC memory (ESP32-C2), also persists to NVS.
*/
void SleepHardware::SetSleepEnterTime() {
  // Capture current time
  int rc = gettimeofday(&sleep_enter_time, NULL);
  if (rc != 0) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to get current time");
    return;
  }

#if !SOC_RTC_FAST_MEM_SUPPORTED
  // For chips without RTC memory (ESP32-C2), persist to NVS
  NvsWriteI32("slp_enter_sec", (int32_t)sleep_enter_time.tv_sec);
  NvsWriteI32("slp_enter_usec", (int32_t)sleep_enter_time.tv_usec);
#endif
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

  sleep_mode = ws_sleep_SleepMode_S_DEEP;
  return true;
}

// TODO: Not implemented yet!
bool SleepHardware::EnableLightSleep(int duration) {
  sleep_mode = ws_sleep_SleepMode_S_LIGHT;
  return true;
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
    @brief  For Light Sleep Mode - Re-enable power to all external components
   that were disabled before entering light sleep (i.e: tft, i2c, neopixel, etc)
*/
void SleepHardware::ReenableExternalComponents() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(TFT_POWER)
  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // (specific to adafruit feather esp32-s2) - turn on the I2C power
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#elif defined(PIN_I2C_POWER)
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#elif defined(TFT_I2C_POWER)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#elif defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif
}

/*!
    @brief  Disables power to all external components that may draw power
            during sleep (i.e: tft, i2c, neopixel, etc)
*/
void SleepHardware::DisableExternalComponents() {
#if defined(TFT_POWER)
  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, LOW);
#endif

#if defined(PIN_NEOPIXEL)
  ReleaseStatusPixel();
#endif

#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // (case specific to only the adafruit feather esp32-s2) - turn off the I2C
  // power by setting pin to rest state (off)
  pinMode(PIN_I2C_POWER, INPUT);
#elif defined(PIN_I2C_POWER)
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, LOW);
#elif defined(TFT_I2C_POWER)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, LOW);
#elif defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif
}

/*!
    @brief  Attempts to stop the WiFi radio to save power before entering light
   sleep.
    @return True if WiFi was successfully stopped, False otherwise.
*/
bool SleepHardware::StopWiFi() {
  esp_err_t rc = esp_wifi_stop();
  if (rc != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to stop WiFi");
    return false;
  }
  return true;
}

/*!
    @brief  Restore WiFi stack persistent settings to default value after waking
   from light sleep.
    @return True if WiFi was successfully started, False otherwise.
*/
bool SleepHardware::RestoreWiFi() {
  esp_err_t rc = esp_wifi_restore();
  if (rc != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to restore WiFi");
    return false;
  }
  return true;
}

/*!
    @brief  Retrieves the current sleep cycle count.
    @return The number of sleep cycles completed.
*/
uint32_t SleepHardware::GetSleepCycleCount() {
#if !SOC_RTC_FAST_MEM_SUPPORTED
  NvsReadU32("cnt_sleep_cycles", &sleep_cycles);
  return sleep_cycles;
#else
  return sleep_cycles;
#endif
}

/*!
    @brief  Retrieves the previous soft RTC counter value from RTC memory.
            On chips without RTC memory (ESP32-C2), reads from NVS.
    @return The stored soft RTC counter value.
*/
uint32_t SleepHardware::GetPrvSoftRtcCounter() {
#if !SOC_RTC_FAST_MEM_SUPPORTED
  uint32_t counter = 0;
  NvsReadU32("cnt_soft_rtc", &counter);
  return counter;
#else
  return cnt_soft_rtc;
#endif
}

/*!
    @brief  Stores the soft RTC counter value to RTC memory.
            On chips without RTC memory (ESP32-C2), persists to NVS.
    @param  counter The counter value to store.
*/
void SleepHardware::StoreSoftRtcCounter(uint32_t counter) {
  cnt_soft_rtc = counter;
#if !SOC_RTC_FAST_MEM_SUPPORTED
  NvsWriteU32("cnt_soft_rtc", counter);
#endif
}

#if !SOC_RTC_FAST_MEM_SUPPORTED
/*!
    @brief  Reads a uint32_t value from NVS storage.
    @param  key   The NVS key to read from (max 15 chars).
    @param  value Pointer to store the read value.
    @return True if read succeeded or key not found, false on error.
*/
bool SleepHardware::NvsReadU32(const char *key, uint32_t *value) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to open NVS for read");
    return false;
  }

  err = nvs_get_u32(nvs_handle, key, value);
  nvs_close(nvs_handle);
  return (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
}

/*!
    @brief  Writes a uint32_t value to NVS storage.
    @param  key   The NVS key to write to (max 15 chars).
    @param  value The value to store.
    @return True if write and commit succeeded, false on error.
*/
bool SleepHardware::NvsWriteU32(const char *key, uint32_t value) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to open NVS for write");
    return false;
  }

  err = nvs_set_u32(nvs_handle, key, value);
  if (err == ESP_OK)
    err = nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  return (err == ESP_OK);
}

/*!
    @brief  Reads an int32_t value from NVS storage.
    @param  key   The NVS key to read from (max 15 chars).
    @param  value Pointer to store the read value.
    @return True if read succeeded or key not found, false on error.
*/
bool SleepHardware::NvsReadI32(const char *key, int32_t *value) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to open NVS for read");
    return false;
  }

  err = nvs_get_i32(nvs_handle, key, value);
  nvs_close(nvs_handle);
  return (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
}

/*!
    @brief  Writes an int32_t value to NVS storage.
    @param  key   The NVS key to write to (max 15 chars).
    @param  value The value to store.
    @return True if write and commit succeeded, false on error.
*/
bool SleepHardware::NvsWriteI32(const char *key, int32_t value) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    WS_DEBUG_PRINTLN("[sleep] ERROR: Failed to open NVS for write");
    return false;
  }

  err = nvs_set_i32(nvs_handle, key, value);
  if (err == ESP_OK)
    err = nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  return (err == ESP_OK);
}
#endif // !SOC_RTC_FAST_MEM_SUPPORTED

#endif // ARDUINO_ARCH_ESP32