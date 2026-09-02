/*!
 * @file src/components/digitalIO/hardware.cpp
 *
 * Hardware driver for the digitalio.proto API
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
#include "hardware.h"

/*!
    @brief  DigitalIOHardware constructor
    @param  pin_name      The pin number.
    @param  direction     The pin's direction.
    @param  sample_mode   The pin's sample mode.
    @param  initial_value The pin's initial value.
    @param  period        The pin's period in milliseconds.
    @param  expander_drv  Pointer to expander driver, or nullptr.
    @param  is_inverted   True if the pin's logic is active-low (inverted).
*/
DigitalIOHardware::DigitalIOHardware(uint8_t pin_name,
                                     ws_digitalio_Direction direction,
                                     ws_digitalio_SampleMode sample_mode,
                                     bool initial_value, ulong period,
                                     ExpanderHardware *expander_drv,
                                     bool is_inverted)
    : _name(pin_name), _direction(direction), _sample_mode(sample_mode),
      _value(initial_value), _prv_value(initial_value), _first_read(true),
      _is_inverted(is_inverted), _period(period), _prv_time(0),
      _did_read_send(false), _expander_drv(expander_drv) {
  SetMode();
}

/*!
    @brief  DigitalIOHardware destructor. Resets pin to floating INPUT state.
*/
DigitalIOHardware::~DigitalIOHardware() {
  bool has_expander = (_expander_drv != nullptr);

  if (!has_expander) {
    digitalWrite(_name, LOW);
    pinMode(_name, INPUT);
  } else {
    _expander_drv->digitalWrite(_name, LOW);
    _expander_drv->pinMode(_name, INPUT);
  }

  if (IsStatusLEDPin())
    initStatusLED();
}

/*!
    @brief  Converts a logical pin value to the electrical level to drive.
            The digitalio.proto `is_inverted` field marks active-low pins
            (e.g. an LED wired to VCC): IO always speaks in logical values,
            so inverted pins must drive the opposite electrical level.
    @param  logical_value  The logical value from the broker's perspective.
    @return HIGH or LOW, the electrical level to apply to the pin.
*/
uint8_t DigitalIOHardware::GetElectricalLevel(bool logical_value) {
  bool level = logical_value;
  if (_is_inverted)
    level = !logical_value;
  if (level)
    return HIGH;
  return LOW;
}

/*!
    @brief  Configures the pin's mode based on its direction.
    @return True if the pin was successfully configured.
*/
bool DigitalIOHardware::SetMode() {
  if (IsStatusLEDPin())
    ReleaseStatusPixel();

  bool has_expander = (_expander_drv != nullptr);

  if (_direction == ws_digitalio_Direction_D_OUTPUT) {
    uint8_t level = GetElectricalLevel(_value);
    if (!has_expander) {
      pinMode(_name, OUTPUT);
      digitalWrite(_name, level);
#if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
      if (level == LOW)
        digitalWrite(_name, HIGH);
#endif
    } else {
      _expander_drv->pinMode(_name, OUTPUT);
      _expander_drv->digitalWrite(_name, level);
    }
  } else if (_direction == ws_digitalio_Direction_D_INPUT) {
    if (!has_expander) {
      pinMode(_name, INPUT);
    } else {
      _expander_drv->pinMode(_name, INPUT);
    }
  } else if (_direction == ws_digitalio_Direction_D_INPUT_PULL_UP) {
    if (!has_expander) {
      pinMode(_name, INPUT_PULLUP);
    } else {
      _expander_drv->pinMode(_name, INPUT_PULLUP);
    }
  } else {
    return false;
  }
  return true;
}

/*!
    @brief  Writes a value to the pin and updates internal state.
    @param  value  The value to write.
*/
void DigitalIOHardware::Write(bool value) {
  if (_value == value)
    return;

  uint8_t level = GetElectricalLevel(value);
  if (_expander_drv != nullptr) {
    _expander_drv->digitalWrite(_name, level);
  } else {
    digitalWrite(_name, level);
  }
  _value = value;
}

/*!
    @brief  Reads the pin's current value from hardware and updates
            internal state.
    @return The pin's current value.
*/
bool DigitalIOHardware::ReadValue() {
  bool raw;
  if (_expander_drv != nullptr) {
    raw = _expander_drv->digitalRead(_name);
  } else {
    raw = digitalRead(_name);
  }

  // Report the logical value: is_inverted (active-low) pins read as the
  // opposite of their electrical level.
  _value = raw;
  if (_is_inverted)
    _value = !raw;
  return _value;
}

/*!
    @brief  Checks if the pin's value has changed since last check.
    @return True if the value changed.
*/
bool DigitalIOHardware::CheckEvent() {
  ReadValue();

  // First read after add: _prv_value was only a guess (the Add's
  // initial_value), so we cannot infer a real transition from it. Comparing
  // against the guess either suppressed the initial report entirely (guess
  // happened to match the real state, IO never learns the pin's value) or
  // fabricated a transition that never happened. Seed the baseline from this
  // read and report it as the pin's initial state instead.
  if (_first_read) {
    _first_read = false;
    _prv_value = _value;
    return true;
  }

  // Has the value changed since the last time we checked?
  if (_value == _prv_value)
    return false;
  _prv_value = _value;
  return true;
}

/*!
    @brief  Checks if the pin's timer has expired and reads the value.
    @return True if the timer expired.
*/
bool DigitalIOHardware::CheckTimer() {
  ulong cur_time = millis();
  if (!IsPinTimerExpired(cur_time))
    return false;

  _prv_time = cur_time;
  ReadValue();
  return true;
}

/*!
    @brief  Checks if the pin's timer has expired.
    @param  cur_time  The current time in milliseconds.
    @return True if the timer has expired.
*/
bool DigitalIOHardware::IsPinTimerExpired(ulong cur_time) {
  return cur_time - _prv_time > _period;
}

/*!
    @brief  Gets the pin's number.
    @return The pin's number.
*/
uint8_t DigitalIOHardware::GetPinNum() const { return _name; }

/*!
    @brief  Gets the pin's current value.
    @return The pin's current value.
*/
bool DigitalIOHardware::GetPinValue() const { return _value; }

/*!
    @brief  Gets the pin's direction.
    @return The pin's direction.
*/
ws_digitalio_Direction DigitalIOHardware::GetDirection() const {
  return _direction;
}

/*!
    @brief  Gets the pin's sample mode.
    @return The pin's sample mode.
*/
ws_digitalio_SampleMode DigitalIOHardware::GetSampleMode() const {
  return _sample_mode;
}

/*!
    @brief  Gets the expander driver, or nullptr for native pins.
    @return Pointer to the expander driver, or nullptr.
*/
ExpanderHardware *DigitalIOHardware::GetExpanderDriver() const {
  return _expander_drv;
}

/*!
    @brief  Gets whether the last read was sent to IO.
    @returns  True if the last read was sent successfully, False otherwise.
*/
bool DigitalIOHardware::DidReadSend() const { return _did_read_send; }

/*!
    @brief  Marks that the current pin value has been sent to IO.
*/
void DigitalIOHardware::MarkSent() { _did_read_send = true; }

/*!
    @brief  Resets the pin's did_read_send flag to false.
*/
void DigitalIOHardware::ResetSendFlag() { _did_read_send = false; }

/*!
    @brief  Checks if this pin is the status LED pin.
    @return True if this pin is the status LED pin.
*/
bool DigitalIOHardware::IsStatusLEDPin() const {
#ifdef STATUS_LED_PIN
  return _name == STATUS_LED_PIN;
#endif
  return false;
}
