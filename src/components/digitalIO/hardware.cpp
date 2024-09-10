#include "hardware.h"

DigitalIOHardware::DigitalIOHardware() {}
DigitalIOHardware::~DigitalIOHardware() {}

void DigitalIOHardware::deinit(uint8_t pin_name) {
  // Turn off pin output and reset mode to hi-z floating state
  digitalWrite(pin_name, LOW);
  pinMode(pin_name, INPUT);
  // TODO: Release status led, if it's a LED, back to the application
}

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

void DigitalIOHardware::SetValue(uint8_t pin_name, bool pin_value) {
  digitalWrite(pin_name, pin_value ? HIGH : LOW);
}

bool DigitalIOHardware::GetValue(uint8_t pin_name) {
  return digitalRead(pin_name);
}