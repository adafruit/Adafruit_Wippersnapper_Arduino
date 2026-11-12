/*!
 * @file src/components/sleep/model.cpp
 *
 * Model for the sleep.proto message.
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
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2350)
#include "model.h"

/*!
    @brief  SleepModel constructor
*/
SleepModel::SleepModel() {
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));
  memset(&_ext0_config, 0, sizeof(_ext0_config));
  _has_ext0 = false;
}

/*!
    @brief  SleepModel destructor
*/
SleepModel::~SleepModel() {
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));
  memset(&_ext0_config, 0, sizeof(_ext0_config));
  _has_ext0 = false;
}

/*!
    @brief  Gets a Sleep Enter message object.
    @return Sleep Enter message object.
*/
ws_sleep_SleepConfig *SleepModel::GetSleepConfig() { return &_msg_sleep_enter; }

/*!
    @brief  Returns whether an ext0 wakeup config has been set.
    @return True if an ext0 wakeup config is present, False otherwise.
*/
bool SleepModel::HasExt0() { return _has_ext0; }

/*!
    @brief  Gets the ext0 wakeup config.
    @return Pointer to the ext0 wakeup config. Only valid if HasExt0().
*/
const ws_sleep_Ext0Config *SleepModel::GetExt0Config() { return &_ext0_config; }

/*!
    @brief  Gets a Sleep Goodnight message object.
    @return Sleep Goodnight message object.
*/
ws_sleep_Goodnight *SleepModel::GetSleepGoodnight() {
  return &_msg_sleep_goodnight;
}

/*!
    @brief  Gets the run duration before sleep, in seconds.
    @return Run duration in seconds.
*/
uint32_t SleepModel::getRunDurationMs() { return _run_duration; }

/*!
    @brief  Decodes the SleepConfig of a Sleep B2D message into the
            _msg_sleep_enter object, and its ext0s into _ext0_config.
    @param  stream
            The nanopb input stream of the Sleep B2D envelope.
    @return True if a SleepConfig was found and decoded.
*/
bool SleepModel::DecodeSleepConfig(pb_istream_t *stream) {
  // Zero-out the Sleep Enter message struct to ensure a clean decode
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));
  SetupExt0Decode(&_msg_sleep_enter);

  // Decode the SleepConfig on its own: decoding the B2D envelope would zero
  // the oneof member, and with it the ext0s decode callback
  return ws_pb_decode_oneof_submsg(stream, ws_sleep_B2D_sleep_config_tag,
                                   ws_sleep_SleepConfig_fields,
                                   &_msg_sleep_enter);
}

/*!
    @brief  Sets up a SleepConfig's ext0s callback field to decode into this
            model, dropping any previously decoded ext0 config. Call before
            decoding the message that holds the SleepConfig.
    @param  msg
            The SleepConfig to decode, e.g. a checkin response's.
*/
void SleepModel::SetupExt0Decode(ws_sleep_SleepConfig *msg) {
  _has_ext0 = false;
  msg->ext0s.funcs.decode = &cbDecodeExt0;
  msg->ext0s.arg = this;
}

/*!
    @brief  Decodes one entry of a SleepConfig's repeated ext0s field.
    @param  stream
            The Ext0Config submessage stream.
    @param  field
            Field descriptor of the ext0s field.
    @param  arg
            Pointer to the SleepModel to decode into.
    @return True if decoded, False otherwise.
*/
bool SleepModel::cbDecodeExt0(pb_istream_t *stream, const pb_field_t *field,
                              void **arg) {
  (void)field;
  SleepModel *model = (SleepModel *)*arg;
  ws_sleep_Ext0Config ext0 = ws_sleep_Ext0Config_init_zero;
  if (!pb_decode(stream, ws_sleep_Ext0Config_fields, &ext0))
    return false;
  // ext0 wakes on a single pin, use the first
  if (model->_has_ext0) {
    WS_DEBUG_PRINTLN("[sleep] WARNING: Only one ext0 pin is used, ignoring");
    return true;
  }
  model->_ext0_config = ext0;
  model->_has_ext0 = true;
  return true;
}

/*!
    @brief  Configures the Sleep Enter message with timer-based wakeup.
    @param  lock
            Whether sleep is locked/enabled.
    @param  mode
            The sleep mode (light/deep).
    @param  run_duration
            Duration to run before sleeping, in seconds.
    @param  timer_duration
            Duration of the sleep timer, in seconds.
*/
void SleepModel::SetSleepEnterTimer(bool lock, const char *mode,
                                    uint32_t run_duration,
                                    uint32_t timer_duration) {
  // Clear the message
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));

  // Set the common fields
  _run_duration = run_duration;

  // Convert strings to enums for mode/wake
  ws_sleep_SleepMode mode_enum = ws_sleep_SleepMode_S_UNSPECIFIED;
  ConvertSleepMode(mode, mode_enum);
  _msg_sleep_enter.mode = mode_enum;

  // Configure timer-specific fields
  _msg_sleep_enter.has_timer = true;
  _msg_sleep_enter.timer.duration = timer_duration;
  _has_ext0 = false;
}

/*!
    @brief  Configures the Sleep Enter message with pin-based wakeup.
    @param  lock
            Whether sleep is locked/enabled.
    @param  mode
            The sleep mode (light/deep).
    @param  run_duration
            Duration to run before sleeping, in seconds.
    @param  pin_name
            Name of the pin to wake from.
    @param  pin_level
            Input level to trigger wakeup (false=low, true=high).
    @note   The pin's pull direction is not set here, it is taken from the
            Digital/Analog component that owns the pin when sleep is
            configured.
*/
void SleepModel::SetSleepEnterExt0(bool lock, const char *mode,
                                   uint32_t run_duration, const char *pin_name,
                                   bool pin_level) {
  // Clear the message
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));

  // Set the common fields
  _run_duration = run_duration;

  // Convert strings to enum for mode
  ws_sleep_SleepMode mode_enum = ws_sleep_SleepMode_S_UNSPECIFIED;
  ConvertSleepMode(mode, mode_enum);
  _msg_sleep_enter.mode = mode_enum;

  // Configure pin-specific fields
  memset(&_ext0_config, 0, sizeof(_ext0_config));
  strncpy(_ext0_config.pin, pin_name, sizeof(_ext0_config.pin) - 1);
  _ext0_config.pin[sizeof(_ext0_config.pin) - 1] = '\0';
  _ext0_config.level = pin_level;
  _has_ext0 = true;
}

/*!
    @brief  Converts sleep mode and wakeup source strings to their corresponding
   enum values.
    @param  mode_str
            The sleep mode string
    @param  mode
            The converted sleep mode enum.
    @returns True if both conversions were successful, False if either string
   was invalid.
*/
void SleepModel::ConvertSleepMode(const char *mode_str,
                                  ws_sleep_SleepMode &mode) {
  // Convert SleepMode to enum
  mode = ws_sleep_SleepMode_S_UNSPECIFIED;
  if (mode_str) {
    if (strncmp(mode_str, "light", strlen("light")) == 0) {
      mode = ws_sleep_SleepMode_S_LIGHT;
    } else if (strncmp(mode_str, "deep", strlen("deep")) == 0) {
      mode = ws_sleep_SleepMode_S_DEEP;
    } else {
      WS_DEBUG_PRINT("[SD] Error: Invalid sleep mode: ");
      WS_DEBUG_PRINTLNVAR(mode_str);
    }
  }
}

/*!
    @brief  Encodes a Sleep Goodnight message into the _msg_sleep_goodnight
            object.
    @param  msg
            Unused - Goodnight is an empty message.
    @return True if the Sleep Goodnight message was successfully encoded.
            False if encoding resulted in a failure.
*/
bool SleepModel::EncodeSleepGoodnight(const char *msg) {
  (void)msg; // Unused - Goodnight is an empty message
  // Initialize the Goodnight message
  _msg_sleep_goodnight = ws_sleep_Goodnight_init_zero;
  return true;
}

#endif // ARDUINO_ARCH_ESP32 || ARDUINO_ARCH_RP2350