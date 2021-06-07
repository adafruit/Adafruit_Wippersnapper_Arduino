/*!
 * @file Wippersnapper_StatusLED.cpp
 *
 * Interfaces for the Wippersnapper status indicator LED/NeoPixel/Dotstar/RGB LED.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "Wippersnapper.h"

#ifdef USE_STATUS_NEOPIXEL
  Adafruit_NeoPixel statusPixel(STATUS_NEOPIXEL_NUM, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

void Wippersnapper::statusLEDInit() {
    #ifdef USE_STATUS_LED
      pinMode(STATUS_LED_PIN, OUTPUT); // Initialize LED
      digitalWrite(STATUS_LED_PIN, 1); // Turn OFF LED
      WS_DEBUG_PRINTLN("Status LED Initialized!");
    #endif

    #ifdef USE_STATUS_NEOPIXEL
      statusPixel.begin();
      statusPixel.clear();
      statusPixel.setBrightness(50);
      statusPixel.show();
      WS_DEBUG_PRINTLN("Status NeoPixel Initialized!");
    #endif
}

void Wippersnapper::statusLEDDeinit() {
  #ifdef USE_STATUS_LED
    digitalWrite(STATUS_LED_PIN, 0); // turn off
    pinMode(pin, INPUT); // "release" by setting to input (hi-z)
  #endif

  #ifdef USE_STATUS_NEOPIXEL
    statusPixel.clear();
    statusPixel.show(); // turn off
    // release for use
    // delete statusPixel;
  #endif
}

void Wippersnapper::setStatusLEDColor(uint32_t color) {
    // TODO: Control brightness as well.
   #ifdef USE_STATUS_NEOPIXEL
    uint8_t red = (color >> 16);   // red
    uint8_t green = (color >> 8);  // green
    uint8_t blue = color;          // blue
    // flood all pixels
    for (int i = 0; i < STATUS_NEOPIXEL_NUM; i++) {
        statusPixel.setPixelColor(i, red, green, blue);
    }
    statusPixel.show();
  #endif
}