// Adafruit IO Wippersnapper Beta
//
// This example is for devices incompatible with TinyUSB, such as the ESP32 and ESP8266
//
// NOTE: This software is a BETA release and in active development.
// Please report bugs or errors to https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/issues
//
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2021
//
// All text above must be included in any redistribution.

#include "Wippersnapper_Networking.h"
Wippersnapper_WiFi wipper;

// Enable debug output for beta builds
#define WS_DEBUG

void setup() {
  Serial.begin(115200);

  // Start the serial connection
  Serial.begin(115200);
  // while(! Serial);

  // Set Adafruit IO Key
  wipper.set_user_key("YOUR_AIO_USERNAME", "YOUR_AIO_PASSWORD");
  // Set WiFi credentials
  wipper.set_ssid_pass("YOUR_WIFI_USERNAME", "YOUR_WIFI_PASSWORD");


  Serial.println("Connecting to Wippersnapper");
  wipper.connect();
  Serial.print("Connected to Wippersnapper!");

}

void loop() {
  wipper.run();
}