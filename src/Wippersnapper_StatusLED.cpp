/*!
 * @file Wippersnapper_Boards.h
 *
 * This file determines hardware/board type at compile-time.
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

bool Wippersnapper::initStatusLED() {
    bool isInitialized = false;
    #if defined(USE_STATUS_LED)
        pinMode(STATUS_LED_PIN, OUTPUT); // Initialize LED
        digitalWrite(STATUS_LED_PIN, 1); // Turn OFF LED
        WS_DEBUG_PRINTLN("Status LED Initialized!");
        isInitialized = true;
    #elif defined(USE_STATUS_NEOPIXEL)
        Adafruit_NeoPixel statusPixel(1, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
        statusPixel.begin();           // initialize neopixel
        statusPixel.clear();            // Turn OFF neopixel
        statusPixel.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
        WS_DEBUG_PRINTLN("Status NeoPixel Initialized!");
        isInitialized = true;
    #else
        isInitialized = false;
    #endif
    // TODO: Add dotstar init sequence */
    return isInitialized;
}
