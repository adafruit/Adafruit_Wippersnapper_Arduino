// Adafruit IO Wippersnapper
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2021
//
// All text above must be included in any redistribution.


#include "config.h"

#define WS_DEBUG // Enable debugging output for testing

void setup() {

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  // while(! Serial);

  // set up wifi iface
  //ws.set_wifi(&SPIWIFI);
  //ws.set_airlift_pins(SPIWIFI_SS, NINA_ACK, NINA_RESETN, NINA_GPIO0);

  // set wifi creds
  ws.set_ssid_pass(WIFI_SSID, WIFI_PASS);

  // set up AIO creds
  ws.set_user_key(IO_USERNAME, IO_KEY);

  Serial.println("Connecting to Wippersnapper");

  ws.connect();

  Serial.print("Connected to Wippersnapper!");

}

void loop() {
  ws.run();
}