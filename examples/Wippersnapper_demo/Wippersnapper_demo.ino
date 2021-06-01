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

  wipper.startProvisioning();
  
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // set up wifi iface
  wipper.set_wifi(&SPIWIFI);
  wipper.set_airlift_pins(SPIWIFI_SS, NINA_ACK, NINA_RESETN, NINA_GPIO0);

  // set wifi creds
  // TODO: this should be handled elsewhere
  wipper.set_ssid_pass(WIFI_SSID, WIFI_PASS);

  // set up AIO creds
  // TODO: this should be handled elsewhere
  wipper.set_user_key(IO_USERNAME, IO_KEY);

  Serial.println("Connecting to Wippersnapper");
  wipper.connect();
  Serial.print("Connected to Wippersnapper!");

}

void loop() {
  wipper.run();
}