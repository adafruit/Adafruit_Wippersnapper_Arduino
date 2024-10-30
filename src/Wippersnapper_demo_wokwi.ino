// Adafruit IO WipperSnapper Beta, Wokwi Test Sketch
//
// ***NOTICE***
// This sketch is not intended to be uploaded to a physical device
// This sketch is for testing Wokwi-CLI and Wokwi-VSCode
//
// Brent Rubell for Adafruit Industries, 2024
//
// All text above must be included in any redistribution.

/************************ Adafruit IO Config *******************************/
// Visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "brubell"
#define IO_KEY "YOUR_AIO_KEY"
/**************************** WiFi Config ***********************************/
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""
// Enable debug output for beta builds
#define WS_DEBUG
// Pin to check for API version
#define API_PIN 0

#include "ws_manager.h"
//#include "Wippersnapper_Networking.h"
//Wippersnapper_WiFi wipper;
Wippersnapper_Manager manager;
Wippersnapper_WiFiV2 wipper(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, "io.adafruit.com", 8883);


void setup() {
  // NOTE: Provisioning must occur prior to serial init.
  manager.checkAPIVersion(API_PIN); 
  manager.provision();

  Serial.begin(115200);
  // while (!Serial) delay(10);
  Serial.println("Adafruit Wippersnapper API Manager Demo");
  Serial.print("Running Wippersnapper API Version: ");
  Serial.println(manager.getAPIVersion());
  manager.connect();
}

void loop() {
  manager.run();
}