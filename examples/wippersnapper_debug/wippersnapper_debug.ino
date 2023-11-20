// Adafruit IO WipperSnapper Beta (DEBUG BUILD ONLY!)
// Brent Rubell for Adafruit Industries, 2021 - 2023

#include "Wippersnapper_Networking.h"
Wippersnapper_WiFi wipper;

// Enable debug output for beta builds
#define WS_DEBUG

void setup() {
  // Provisioning must occur prior to serial init.
  wipper.provision();

  Serial.begin(115200); // wippersnapper serial
  Serial1.begin(115200);  // ESP-IDF messages serial
  Serial1.setDebugOutput(true); // Enable ESP-IDF messages over Serial1
  //while (!Serial) delay(10);

  wipper.connect();
}

void loop() {
  wipper.run();
}