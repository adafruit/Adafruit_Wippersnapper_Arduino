// Adafruit IO WipperSnapper Beta
//
//
// NOTE: This software is a BETA release and in active development.
// Please report bugs or errors to https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/issues
//
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2021-2022
//
// All text above must be included in any redistribution.

#include "ws_manager.h"
//#include "Wippersnapper_Networking.h"
//Wippersnapper_WiFi wipper;
Wippersnapper_Manager manager;

// Enable debug output for beta builds
#define WS_DEBUG

void setup() {


  // TODO: Call manager instance instead
  // Provisioning must occur prior to serial init.
  // wipper.provision();

  Serial.begin(115200);
  while(!Serial) {;}
  Serial.println("Adafruit Wippersnapper API Manager Demo");
  // Check API version
  Serial.println("Checking API Version...");
  manager.checkAPIVersion();
  Serial.print("Wippersnapper API Version:");
  Serial.println(manager.getAPIVersion());

  // wipper.connect();

}

void loop() {
  //wipper.run();
}