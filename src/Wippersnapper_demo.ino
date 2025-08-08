// Adafruit IO WipperSnapper
// USE ONLY WITH DEVICES WITHOUT A NETWORK ADAPTER LIKE RP2040 PICO
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2025
//
// All text above must be included in any redistribution.

#include "ws_adapters.h"
#if defined(OFFLINE_MODE_WOKWI)
ws_adapter_wifi wipper; // Wokwi offline mode uses a wifi adapter
#elif defined(WS_WIFI_ADAPTER)
ws_adapter_wifi wipper;
#elif defined(WS_OFFLINE_ADAPTER)
ws_adapter_offline wipper;
#else
#error "No valid ws_adapter_wifi or ws_adapter_offline defined! Please check your board configuration."
#endif
#define WS_DEBUG // Enable debug output!

void setup() {
  Serial.begin(115200);
  while(!Serial);
  wipper.provision();
  wipper.connect();
}

void loop() {
  wipper.run();
}