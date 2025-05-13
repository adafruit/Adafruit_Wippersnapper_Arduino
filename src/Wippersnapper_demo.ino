// Adafruit IO WipperSnapper
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2020-2025
//
// All text above must be included in any redistribution.

#include "ws_adapters.h"
#if defined(ARDUINO_RASPBERRY_PI_PICO_2) ||                                  \
    defined(ARDUINO_RASPBERRY_PI_PICO) ||                                      \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER) ||                      \
    defined(ARDUINO_ADAFRUIT_METRO_RP2350)
ws_adapter_offline wipper;
#else
ws_adapter_wifi wipper;
#endif

#define WS_DEBUG // Enable debug output!

void setup() {
  Serial.begin(115200);
  wipper.provision();
  wipper.connect();
}

void loop() {
  wipper.run();
}