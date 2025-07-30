/*!
 * @file src/components/digitalIO/hardware.cpp
 *
 * Hardware driver for the digitalio.proto API
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
bool DigitalIOHardware::ConfigurePin(
    uint8_t name, wippersnapper_digitalio_DigitalIODirection direction) {
  // Configure an output pin
  if (direction ==
      wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_OUTPUT) {
    // Set pin mode to OUTPUT
    pinMode(name, OUTPUT);
// Initialize pin value to LOW
#if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
    // The Adafruit Feather ESP8266's built-in LED is reverse wired so setting
    // the pin LOW will turn the LED on.
    digitalWrite(name, !0);
#else
    pinMode(name, OUTPUT);
    digitalWrite(name, LOW); // initialize LOW
#endif
  } else if (
      direction ==
      wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT) {
    pinMode(name, INPUT);
  } else if (
      direction ==
      wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT_PULL_UP) {
    pinMode(name, INPUT_PULLUP);
  } else {
    return false; // Invalid pin configuration
  }
  return true;
}

/*!
    @brief  Deinitializes a digital pin.
    @param  pin_name
            The digital pin to deinitialize.
*/
void DigitalIOHardware::deinit(uint8_t pin_name) {
  // Turn off pin output and reset mode to hi-z floating state
  digitalWrite(pin_name, LOW);
  pinMode(pin_name, INPUT);
  // Prior to using this pin as a DIO,
  // was this a status LED pin?
  if (IsStatusLEDPin(pin_name)) {
    initStatusLED(); // it was! re-init status led
  }
}

/*!
    @brief  Sets a digital pin's value.
    @param  pin_name
            The pin's name.
    @param  pin_value
            The pin's value.
*/
void DigitalIOHardware::SetValue(uint8_t pin_name, bool pin_value) {
  digitalWrite(pin_name, pin_value ? HIGH : LOW);
}

/*!
    @brief  Gets a digital pin's value.
    @param  pin_name
            The pin's name.
    @return The pin's value.
*/
bool DigitalIOHardware::GetValue(uint8_t pin_name) {
  return digitalRead(pin_name);
}

/*!
    @brief  Checks if a pin is the status LED pin.
    @param  pin_name
            The pin's name.
    @return True if the pin is the status LED pin.
*/
bool DigitalIOHardware::IsStatusLEDPin(uint8_t pin_name) {
#ifdef STATUS_LED_PIN
  return pin_name == STATUS_LED_PIN;
#endif
  return false;
}