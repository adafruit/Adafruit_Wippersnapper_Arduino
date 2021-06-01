// Adafruit IO Wippersnapper
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2020
//
// All text above must be included in any redistribution.


#include "config.h"

#define WS_DEBUG

void setup() {

  #ifdef USE_TINYUSB
    wipper.startProvisioning();
  #endif

  Serial.begin(115200);
  while (!Serial) delay(10);

  #ifdef USE_TINYUSB
    // Use native usb provisioning
    // Validate
    wipper.validateProvisioningSecrets();
    // Set credentials and setup from secrets.json file
    wipper.parseProvisioningSecrets();
    // Set Adafruit IO credentials
    wipper.set_user_key();
    // Set WiFi Credentials
    wipper.set_ssid_pass(WIFI_SSID, WIFI_PASS);
  #else
    // non-native USB workflow
    wipper.set_user_key(IO_USERNAME, IO_KEY);
    wipper.set_ssid_pass(WIFI_SSID, WIFI_PASS);
  #endif

  // TODO - this should be handled by the secrets.json as well!
  // Configure WiFi network iface (AirLift only)
  wipper.set_wifi(&SPIWIFI);
  wipper.set_airlift_pins(SPIWIFI_SS, NINA_ACK, NINA_RESETN, NINA_GPIO0);

  Serial.println("Connecting to Wippersnapper");
  wipper.connect();
  Serial.print("Connected to Wippersnapper!");

}

void loop() {
  wipper.run();
}