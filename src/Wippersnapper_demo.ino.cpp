# 1 "/var/folders/ff/dmzflvf52tq9kzvt6g8jglxw0000gn/T/tmp5gn0ambf"
#include <Arduino.h>
# 1 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
# 11 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
#include "ws_adapters.h"
ws_adapter_wifi wipper;


#define WS_DEBUG
void setup();
void loop();
#line 17 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  wipper.provision();
  wipper.connect();
}

void loop() { wipper.run(); }