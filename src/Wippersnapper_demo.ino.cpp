# 1 "/var/folders/ff/dmzflvf52tq9kzvt6g8jglxw0000gn/T/tmpk62p1kwv"
#include <Arduino.h>
# 1 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
# 11 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
#include "ws_adapters.h"
#if defined(ARDUINO_RASPBERRY_PI_PICO_2) || \
    defined(ARDUINO_RASPBERRY_PI_PICO) || \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER) || \
    defined(ARDUINO_ADAFRUIT_METRO_RP2350)
ws_adapter_offline wipper;
#else
ws_adapter_wifi wipper;
#endif

#define WS_DEBUG
void setup();
void loop();
#line 23 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
void setup() {
  Serial.begin(115200);
  wipper.provision();
  wipper.connect();
}

void loop() {
  wipper.run();
}