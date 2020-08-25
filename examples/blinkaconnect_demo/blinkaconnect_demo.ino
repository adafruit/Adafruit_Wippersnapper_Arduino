// WipperSnapper Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2020
//
// All text above must be included in any redistribution.


#include "config.h"

void setup() {

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit WipperSnapper");
  
  bc.connect();

}

void loop() {
  bc.run();
}
