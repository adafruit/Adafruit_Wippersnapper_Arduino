/*!
 * @file ws_helper_esp.h
 *
 * This file contains helper functions for the ESPx platforms.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_HELPER_ESP_H
#define WS_HELPER_ESP_H
#include "esp_task_wdt.h"
#include <Esp.h>

/*******************************************************************************************************************************/
/*!
    @brief    Converts reset reason type to a C string.. This uses the mechanism
   in IDF that persists crash reasons across a reset. see:
   https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html#software-reset
    @param    r
              The reset reason, esp_reset_reason_t
    @return   A C string representing the reset reason.
*/
/*******************************************************************************************************************************/
inline const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
  case ESP_RST_UNKNOWN:
    return "Unknown";
  case ESP_RST_POWERON:
    return "PowerOn"; // Power on or RST pin toggled
  case ESP_RST_EXT:
    return "ExtPin"; // External pin - not applicable for ESP32
  case ESP_RST_SW:
    return "Reboot"; // esp_restart()
  case ESP_RST_PANIC:
    return "Crash"; // Exception/panic
  case ESP_RST_INT_WDT:
    return "WDT_Int"; // Interrupt watchdog (software or hardware)
  case ESP_RST_TASK_WDT:
    return "WDT_Task"; // Task watchdog
  case ESP_RST_WDT:
    return "WDT_Other"; // Other watchdog
  case ESP_RST_DEEPSLEEP:
    return "Sleep"; // Reset after exiting deep sleep mode
  case ESP_RST_BROWNOUT:
    return "BrownOut"; // Brownout reset (software or hardware)
  case ESP_RST_SDIO:
    return "SDIO"; // Reset over SDIO
  default:
    return "";
  }
}
#endif // WS_HELPER_ESP_H
