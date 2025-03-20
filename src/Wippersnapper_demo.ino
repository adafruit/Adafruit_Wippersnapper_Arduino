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
//ws_adapter_wifi wipper;
// Uncomment the following line to use the offline adapter for Pico
ws_adapter_offline wipper;
#define WS_DEBUG // Enable debug output!

void setup() {
  Serial.begin(115200);
  wipper.provision();
  wipper.connect();
}

void loop() { wipper.run(); }