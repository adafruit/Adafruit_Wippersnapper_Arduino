// Adafruit IO WipperSnapper - FULL OFFLINE BUILD for Devices w/o a Network Connection
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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  wipper.provision();
  wipper.connect();
}

void loop() {
  wipper.run();
}