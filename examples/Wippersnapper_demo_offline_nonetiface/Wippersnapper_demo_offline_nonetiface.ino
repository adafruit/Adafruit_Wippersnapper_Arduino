// Adafruit IO WipperSnapper - Offline Mode
// USE ONLY WITH DEVICES WITH A NETWORK ADAPTER LIKE ESP32-x
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2025
//
// All text above must be included in any redistribution.

#include "ws_adapters.h"
ws_adapter_offline wipper;
#define WS_DEBUG // Enable debug output!
#define BUILD_OFFLINE_ONLY

void setup() {
  Serial.begin(115200);
  wipper.provision();
  wipper.connect();
}

void loop() {
  wipper.run();
}