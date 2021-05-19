// Adafruit IO Wippersnapper Beta
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
// Brent Rubell for Adafruit Industries, 2021
//
// All text above must be included in any redistribution.


#include "config.h"

// Enable debugging output for beta testers
#define WS_DEBUG

void setup() {

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  // while(! Serial);

  // AirLift ONLY!
  // TODO: This should be handled elsewhere
  // set up wifi iface
  //ws.set_wifi(&SPIWIFI);
  //ws.set_airlift_pins(SPIWIFI_SS, NINA_ACK, NINA_RESETN, NINA_GPIO0);

  // set wifi creds
  // TODO: this should be handled elsewhere
  ws.set_ssid_pass(WIFI_SSID, WIFI_PASS);

  // set up AIO creds
  // TODO: this should be handled elsewhere
  ws.set_user_key(IO_USERNAME, IO_KEY);

  Serial.println("Connecting to Wippersnapper");
  ws.connect();
  Serial.print("Connected to Wippersnapper!");

}

// Do NOT add code to, or modify, this blocking loop.
void loop() {
  ws.run();
}