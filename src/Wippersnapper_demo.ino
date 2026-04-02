// Adafruit IO WipperSnapper
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2026
//
// All text above must be included in any redistribution.

#include "ws_platforms.h"
#if defined(PICO_CYW43_SUPPORTED) || defined(ARDUINO_ARCH_ESP8266) || defined(USE_AIRLIFT) \
    || defined(ARDUINO_ARCH_ESP32) || defined(ESP_HOSTED)
ws_adapter_wifi wipper;
#else
// Uncomment the following line to use the offline adapter for Pico
ws_adapter_offline wipper;
#endif
#define WS_DEBUG // Enable debug output!

void setup() {
  Serial.begin(115200);
  wipper.provision();
  wipper.connect();
}

void loop() { wipper.run(); }