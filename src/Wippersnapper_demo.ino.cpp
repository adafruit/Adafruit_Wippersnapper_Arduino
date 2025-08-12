# 1 "C:\\Users\\tyeth\\AppData\\Local\\Temp\\tmp97kub74s"
#include <Arduino.h>
# 1 "C:/dev/arduino/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
# 12 "C:/dev/arduino/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
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
#line 24 "C:/dev/arduino/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
void setup() {
  Serial.begin(115200);
  while(!Serial);
  wipper.provision();
  wipper.connect();
}

void loop() {
  wipper.run();
}