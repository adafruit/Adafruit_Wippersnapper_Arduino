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
*/
DigitalIOHardware::DigitalIOHardware() {}

/*!
    @brief  DigitalIOHardware destructor
*/
DigitalIOHardware::~DigitalIOHardware() {}

/*!
    @brief  Configures a digital pin.
    @param  name
            The pin's name.
    @param  direction
            The pin's direction.
    @return True if the pin was successfully configured. False otherwise.
*/
bool DigitalIOHardware::SetPinMode(DigitalIOPin *pin) {
  if (pin == nullptr)
    return false;

  bool has_expander = (pin->expander_drv != nullptr);

  // Configure an output pin
  if (pin->pin_direction == ws_digitalio_Direction_D_OUTPUT) {
    // Set pin mode to OUTPUT
    if (!has_expander) {
      pinMode(pin->pin_name, OUTPUT);
      digitalWrite(pin->pin_name, LOW); // initialize LOW
#if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
      // The Adafruit Feather ESP8266's built-in LED is reverse wired so
      // setting the pin LOW will turn the LED on.
      digitalWrite(pin->pin_name, !0);
#endif
    } else {
      pin->expander_drv->pinMode(pin->pin_name, OUTPUT);
      pin->expander_drv->digitalWrite(pin->pin_name, LOW); // initialize LOW
    }
  } else if (pin->pin_direction == ws_digitalio_Direction_D_INPUT) {
    if (!has_expander) {
      pinMode(pin->pin_name, INPUT);
    } else {
      pin->expander_drv->pinMode(pin->pin_name, INPUT);
    }
  } else if (pin->pin_direction == ws_digitalio_Direction_D_INPUT_PULL_UP) {
    if (!has_expander) {
      pinMode(pin->pin_name, INPUT_PULLUP);
    } else {
      pin->expander_drv->pinMode(pin->pin_name, INPUT_PULLUP);
    }
  } else {
    return false; // Invalid pin configuration
  }
  return true;
}

/*!
    @brief  Deinitializes a digital pin.
    @param  pin
            The digital pin to deinitialize.
*/
void DigitalIOHardware::deinit(DigitalIOPin *pin) {
  if (pin == nullptr)
    return;

  bool has_expander = (pin->expander_drv != nullptr);

  // Turn off pin output and reset mode to hi-z floating state
  if (!has_expander) {
    digitalWrite(pin->pin_name, LOW);
    pinMode(pin->pin_name, INPUT);
  } else {
    pin->expander_drv->digitalWrite(pin->pin_name, LOW);
    pin->expander_drv->pinMode(pin->pin_name, INPUT);
  }
  // Prior to using this pin as a DIO,
  // was this a status LED pin?
  if (IsStatusLEDPin(pin)) {
    initStatusLED(); // it was! re-init status led
  }
}

/*!
    @brief  Sets a digital pin's value.
    @param  pin
            The digital pin.
    @param  pin_value
            The pin's value.
*/
void DigitalIOHardware::SetValue(DigitalIOPin *pin, bool pin_value) {
  if (pin == nullptr)
    return;

  bool has_expander = (pin->expander_drv != nullptr);

  if (!has_expander) {
    digitalWrite(pin->pin_name, pin_value ? HIGH : LOW);
  } else {
    pin->expander_drv->digitalWrite(pin->pin_name, pin_value ? HIGH : LOW);
  }
}

/*!
    @brief  Gets a digital pin's value.
    @param  pin
            The digital pin.
    @return The pin's value.
*/
bool DigitalIOHardware::GetValue(DigitalIOPin *pin) {
  if (pin == nullptr)
    return false;

  bool has_expander = (pin->expander_drv != nullptr);

  if (!has_expander) {
    return digitalRead(pin->pin_name);
  } else {
    return pin->expander_drv->digitalRead(pin->pin_name);
  }
}

/*!
    @brief  Checks if a pin is the status LED pin.
    @param  pin
            The digital pin.
    @return True if the pin is the status LED pin.
*/
bool DigitalIOHardware::IsStatusLEDPin(DigitalIOPin *pin) {
  if (pin == nullptr)
    return false;
#ifdef STATUS_LED_PIN
  return pin->pin_name == STATUS_LED_PIN;
#endif
  return false;
}