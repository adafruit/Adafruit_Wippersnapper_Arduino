#include "hardware.h"

DigitalIOHardware::DigitalIOHardware() {}
DigitalIOHardware::~DigitalIOHardware() {}

void DigitalIOHardware::deinit(uint8_t pin_name) {
  // Turn off pin output and reset mode to hi-z floating state
  digitalWrite(pin_name, LOW);
  pinMode(pin_name, INPUT);
  // TODO: Release status led, if it's a LED, back to the application
}

bool DigitalIOHardware::ConfigurePin(uint8_t pin_name, bool is_output,
                                     bool has_pullups) {
  // Configure an output pin
  if (is_output) {
    // Set pin mode to OUTPUT
    pinMode(pin_name, OUTPUT);
// Initialize pin value to LOW
#if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
    // The Adafruit Feather ESP8266's built-in LED is reverse wired so setting
    // the pin LOW will turn the LED on.
    digitalWrite(pin_name, !0);
#else
    pinMode(pin_name, OUTPUT);
    digitalWrite(pin_name, LOW); // initialize LOW
#endif
  } else if (!is_output && !has_pullups) {
    pinMode(pin_name, INPUT);
  } else if (!is_output && has_pullups) {
    pinMode(pin_name, INPUT_PULLUP);
  } else {
    return false; // Invalid pin configuration
  }
  return true;
}

void DigitalIOHardware::WriteDigitalPin(uint8_t pin_name, bool pin_value) {
  if (pin_value == true)
    digitalWrite(pin_name, HIGH);
  else
    digitalWrite(pin_name, LOW);
}
