/*!
 * @file drvBase.cpp
 *
 * Base implementation for I2C device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "drvBase.h"
#include "../model.h"

/*!
    @brief    Applies settings to the driver.
    @param    settings
              Pointer to the decoded settings array, or nullptr.
    @param    count
              The number of settings in the array.
    @returns  True if all settings (or the defaults) were applied
              successfully, False if any setting key was unsupported or if a
   setting failed to apply.
*/
bool drvBase::configure(DecodedSetting *settings, size_t count) {
  // No broker-provided settings, so apply the driver's default configuration
  if (settings == nullptr || count == 0) {
    return configureDefaults();
  }

  bool success = true;
  // Walk the settings and apply them to the driver
  for (size_t i = 0; i < count; i++) {
    if (!settings[i].has_value) {
      continue;
    }

    if (strcmp(settings[i].key, "gain") == 0) {
      if (settings[i].which_value == ws_config_Value_int_value_tag) {
        if (!setGain(settings[i].int_value))
          success = false;
      }
    } else if (strcmp(settings[i].key, "light_gain") == 0) {
      if (settings[i].which_value == ws_config_Value_int_value_tag) {
        if (!setLightGain(settings[i].int_value))
          success = false;
      }
    } else if (strcmp(settings[i].key, "light_resolution") == 0) {
      if (settings[i].which_value == ws_config_Value_int_value_tag) {
        if (!setLightResolution(settings[i].int_value))
          success = false;
      }
    } else if (strcmp(settings[i].key, "prox_resolution") == 0) {
      if (settings[i].which_value == ws_config_Value_int_value_tag) {
        if (!setProxResolution(settings[i].int_value))
          success = false;
      }
    } else if (strcmp(settings[i].key, "light_meas_rate") == 0) {
      if (settings[i].which_value == ws_config_Value_int_value_tag) {
        if (!setLightMeasRate(settings[i].int_value))
          success = false;
      }
    } else if (strcmp(settings[i].key, "prox_meas_rate") == 0) {
      if (settings[i].which_value == ws_config_Value_int_value_tag) {
        if (!setProxMeasRate(settings[i].int_value))
          success = false;
      }
    } else {
      // Unknown key - report failure but keep applying the remaining settings
      success = false;
    }
  }

  return success;
}
