# 1 "/var/folders/ff/dmzflvf52tq9kzvt6g8jglxw0000gn/T/tmpurmcpxg_"
#include <Arduino.h>
# 1 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo_wokwi.ino"
# 14 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo_wokwi.ino"
#define IO_USERNAME "brubell"
#define IO_KEY "YOUR_AIO_KEY"

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""

#define WS_DEBUG 

#define API_PIN 0

#include "ws_manager.h"


Wippersnapper_Manager manager;
Wippersnapper_WiFiV2 wipper(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, "io.adafruit.com", 8883);
void setup();
void loop();
#line 31 "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/src/Wippersnapper_demo_wokwi.ino"
void setup() {

  manager.checkAPIVersion(API_PIN);
  manager.provision();

  Serial.begin(115200);

  Serial.println("Adafruit Wippersnapper API Manager Demo");
  Serial.print("Running Wippersnapper API Version: ");
  Serial.println(manager.getAPIVersion());
  manager.connect();
}

void loop() {
  manager.run();
}