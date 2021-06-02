// Adafruit IO Wippersnapper
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2020
//
// All text above must be included in any redistribution.
#include "Wippersnapper_Networking.h"
Wippersnapper_WiFi wipper;

// Enable debug output for beta builds
#define WS_DEBUG

void setup() {
  // Provisioning must occur prior to serial init.
  #ifdef USE_TINYUSB
    wipper.startProvisioning();
  #endif

  Serial.begin(115200);
  while (!Serial) delay(10);

  #ifdef USE_TINYUSB
    // Validate secrets file exists
    wipper.validateProvisioningSecrets();
    // Parse out secrets file
    wipper.parseProvisioningSecrets();
    // Set Adafruit IO credentials
    wipper.set_user_key();
    // Set WiFi credentials
    wipper.set_ssid_pass();
  #else // non-native USB workflow
    // Set Adafruit IO Key
    wipper.set_user_key("YOUR_IO_USERNAME", "YOUR_IO_KEY");
    // Set WiFi credentials
    wipper.set_ssid_pass("YOUR_WIFI_SSID", YOUR_WIFI_PASS");
  #endif

  Serial.println("Connecting to Wippersnapper");
  wipper.connect();
  Serial.print("Connected to Wippersnapper!");

}

void loop() {
  wipper.run();
}