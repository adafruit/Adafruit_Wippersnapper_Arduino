# 1 "/var/folders/ff/dmzflvf52tq9kzvt6g8jglxw0000gn/T/tmpuwpc59cm"
#include <Arduino.h>
# 1 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
# 16 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
#include "ws_manager.h"


Wippersnapper_Manager manager;


#define WS_DEBUG 


#define API_PIN 0
void setup();
void loop();
#line 27 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo.ino"
void setup() {

  manager.checkAPIVersion(API_PIN);
  manager.provision();

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit Wippersnapper API Manager Demo");
  Serial.print("Running Wippersnapper API Version: ");
  Serial.println(manager.getAPIVersion());
  manager.connect();
}

void loop() {
  manager.run();
}