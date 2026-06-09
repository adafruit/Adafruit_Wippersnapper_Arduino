/*!
 * @file drvBase.cpp
 *
 * Base implementation for I2C device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "drvBase.h"
#include "../model.h"

/*!
    @brief  Pointer to a member function that sets a driver configuration value.
*/
typedef bool (drvBase::*SetterFn)(const ws_config_Value &);

struct SettingHandler {
  const char *key; ///< Key (e.g. "gain", "mode"), from the broker
  SetterFn setter; ///< Desired function to invoke, on the driver
}; ///< Maps broker setting key to the driver's setter function

static const SettingHandler kSettingHandlers[] = {
    {"gain", &drvBase::setGain},
    {"light_gain", &drvBase::setLightGain},
    {"light_resolution", &drvBase::setLightResolution},
    {"prox_resolution", &drvBase::setProxResolution},
    {"light_meas_rate", &drvBase::setLightMeasRate},
    {"prox_meas_rate", &drvBase::setProxMeasRate},
    {"integration_time", &drvBase::setIntegrationTime},
    {"measurement_rate", &drvBase::setMeasurementRate},
    {"averaged_samples", &drvBase::setAveragedSamples},
    {"read_delay", &drvBase::setReadDelay},
    {"lux_method", &drvBase::setLuxMethod},
    {"temp_oversampling", &drvBase::setTempOversampling},
    {"pressure_oversampling", &drvBase::setPressureOversampling},
    {"humidity_oversampling", &drvBase::setHumidityOversampling},
    {"iir_filter", &drvBase::setIirFilter},
    {"output_data_rate", &drvBase::setOutputDataRate},
    {"mode", &drvBase::setMode},
    {"filter", &drvBase::setFilter},
    {"standby", &drvBase::setStandby},
    {"calibration", &drvBase::setCalibration},
}; ///< Maps a broker-provided key to a setter function in the driver

/*!
    @brief  Applies configuration settings from the broker to the driver by
            dispatching to the appropriate setter function for each setting.
*/
bool drvBase::configure(DecodedSetting *settings, size_t count) {
  // No settings provided, so apply the driver's default configuration
  if (settings == nullptr || count == 0) {
    return configureDefaults();
  }

  bool success = true;
  // Walk the settings and dispatch each to its driver setter
  for (size_t i = 0; i < count; i++) {
    if (!settings[i].has_value) {
      continue;
    }

    bool found = false;
    for (const SettingHandler &handler : kSettingHandlers) {
      if (strcmp(settings[i].key, handler.key) != 0) {
        continue;
      }
      // If the setting key was recognized but the value was invalid or failed
      // to apply, log an error but continue processing remaining settings
      if (!(this->*handler.setter)(DecodedSettingToValue(settings[i]))) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to apply setting: %s",
                 settings[i].key);
        ws_i2c_Descriptor descriptor = ws_i2c_Descriptor_init_zero;
        descriptor.address = _address;
        Ws.error_handler->publishComponentError(descriptor, msg);
        success = false;
      }
      found = true;
      break;
    }

    // If the setting key was unrecognized, log an error but continue processing
    // remaining settings
    if (!found) {
      char msg[64];
      snprintf(msg, sizeof(msg), "Unknown setting key: %s", settings[i].key);
      ws_i2c_Descriptor descriptor = ws_i2c_Descriptor_init_zero;
      descriptor.address = _address;
      Ws.error_handler->publishComponentError(descriptor, msg);
      success = false;
    }
  }

  return success;
}
