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
}

/*!
    @brief  SleepModel destructor
*/
SleepModel::~SleepModel() {
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));
}

/*!
    @brief  Gets a Sleep Enter message object.
    @return Sleep Enter message object.
*/
ws_sleep_SleepConfig *SleepModel::GetSleepConfig() { return &_msg_sleep_enter; }

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
    @brief  Decodes a Sleep Enter message into the _msg_sleep_enter
            object from a nanopb stream.
    @param  stream
            The nanopb input stream.
    @return True if the Sleep Enter message was successfully decoded.
*/
bool SleepModel::DecodeSleepConfig(pb_istream_t *stream) {
  // Zero-out the Sleep Enter message struct to ensure a clean decode
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));

  // Decode the stream into a Sleep Enter message
  return pb_decode(stream, ws_sleep_SleepConfig_fields, &_msg_sleep_enter);
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
  _msg_sleep_enter.which_config = ws_sleep_SleepConfig_timer_tag;
  _msg_sleep_enter.config.timer.duration = timer_duration;
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
    @param  pin_pull
            Enable internal pull resistor.
*/
void SleepModel::SetSleepEnterExt0(bool lock, const char *mode,
                                   uint32_t run_duration, const char *pin_name,
                                   bool pin_level, bool pin_pull) {
  // Clear the message
  memset(&_msg_sleep_enter, 0, sizeof(_msg_sleep_enter));

  // Set the common fields
  _run_duration = run_duration;

  // Convert strings to enum for mode
  ws_sleep_SleepMode mode_enum = ws_sleep_SleepMode_S_UNSPECIFIED;
  ConvertSleepMode(mode, mode_enum);
  _msg_sleep_enter.mode = mode_enum;

  // Configure pin-specific fields
  _msg_sleep_enter.which_config = ws_sleep_SleepConfig_ext0_tag;
  strncpy(_msg_sleep_enter.config.ext0.pin_name, pin_name,
          sizeof(_msg_sleep_enter.config.ext0.pin_name) - 1);
  _msg_sleep_enter.config.ext0
      .pin_name[sizeof(_msg_sleep_enter.config.ext0.pin_name) - 1] = '\0';
  _msg_sleep_enter.config.ext0.level = pin_level;
  _msg_sleep_enter.config.ext0.pull = pin_pull;
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
            object. The Goodnight message is now empty (no fields).
    @return True if the Sleep Goodnight message was successfully encoded.
            False if encoding resulted in a failure.
*/
bool SleepModel::EncodeSleepGoodnight() {
  memset(&_msg_sleep_goodnight, 0, sizeof(_msg_sleep_goodnight));

  size_t sz_goodnight_msg;
  if (!pb_get_encoded_size(&sz_goodnight_msg, ws_sleep_Goodnight_fields,
                           &_msg_sleep_goodnight))
    return false;

  uint8_t buf[sz_goodnight_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_sleep_Goodnight_fields,
                   &_msg_sleep_goodnight);
}

#endif // ARDUINO_ARCH_ESP32 || ARDUINO_ARCH_RP2350