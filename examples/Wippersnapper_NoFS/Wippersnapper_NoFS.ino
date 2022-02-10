// Adafruit IO WipperSnapper Beta
//
//
// NOTE: This software is a BETA release and in active development.
// Please report bugs or errors to
// https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/issues
//
// This sketch is for devices which lack USB-MSD or LittleFS support such
// as the Arduino MKR WiFi 1010, Arduino Nano 33 IoT.
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2021
//
// All text above must be included in any redistribution.

#include "Wippersnapper_Networking.h"

/************************ Adafruit IO Config *******************************/

// Visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "YOUR_AIO_USERNAME"
#define IO_KEY "YOUR_AIO_KEY"

/**************************** WiFi Config ***********************************/
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

#include "Wippersnapper_Networking.h"
Wippersnapper_WiFi wipper(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

void setup() {
  // Provisioning must occur prior to serial init.
  wipper.provision();

  Serial.begin(115200);
  // while (!Serial) delay(10);

  wipper.connect();
}

void loop() { wipper.run(); }
