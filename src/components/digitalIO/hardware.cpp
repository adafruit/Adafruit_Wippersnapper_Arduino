#include "hardware.h"

DigitalIOHardware::DigitalIOHardware() {}
DigitalIOHardware::~DigitalIOHardware() {}

void DigitalIOHardware::SetPinMode(uint8_t pin_name, bool is_output,
                                   bool has_pullups) {
  // TODO: Handle if pin is the status LED
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
    return;
  }
  // TODO: Handle input
  // TODO: Handle input with pullups
}
