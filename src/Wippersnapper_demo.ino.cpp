# 1 "/var/folders/ff/dmzflvf52tq9kzvt6g8jglxw0000gn/T/tmpf500i4mg"
#include <Arduino.h>
# 1 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
# 12 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
#include "ws_adapters.h"
#if defined(OFFLINE_MODE_WOKWI)
ws_adapter_wifi wipper;
#elif defined(WS_WIFI_ADAPTER)
ws_adapter_wifi wipper;
#elif defined(WS_OFFLINE_ADAPTER)
ws_adapter_offline wipper;
#else
#error "No valid ws_adapter_wifi or ws_adapter_offline defined! Please check your board configuration."
#endif
#define WS_DEBUG
void setup();
void loop();
#line 24 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("WipperSnapper Starting...");
  wipper.provision();
  Serial.println("WipperSnapper Connecting...");
  wipper.connect();
}

void loop() {
  wipper.run();
}