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

#ifdef USE_STATUS_DOTSTAR
  Adafruit_DotStar statusPixelDotStar(STATUS_DOTSTAR_NUM, STATUS_DOTSTAR_PIN_DATA, STATUS_DOTSTAR_PIN_CLK, DOTSTAR_BRG);
#endif

/****************************************************************************/
/*!
    @brief    Initializes board-specific status LED.
*/
/****************************************************************************/
void Wippersnapper::statusLEDInit() {
    #ifdef USE_STATUS_LED
      pinMode(STATUS_LED_PIN, OUTPUT); // Initialize LED
      digitalWrite(STATUS_LED_PIN, 1); // Turn OFF LED
    #endif

    #ifdef USE_STATUS_NEOPIXEL
      statusPixel.begin();
      statusPixel.setBrightness(50);
      statusPixel.clear();
      statusPixel.show();
    #endif

    #ifdef USE_STATUS_DOTSTAR
      statusPixelDotStar.begin();
      statusPixelDotStar.setBrightness(50);
      statusPixelDotStar.clear();
      statusPixelDotStar.show();
    #endif
}

/****************************************************************************/
/*!
    @brief    De-initializes board-specific status LED.
*/
/****************************************************************************/
void Wippersnapper::statusLEDDeinit() {
  #ifdef USE_STATUS_LED
    digitalWrite(STATUS_LED_PIN, 0); // turn off
    pinMode(pin, INPUT); // "release" by setting to input (hi-z)
  #endif

  #ifdef USE_STATUS_NEOPIXEL
    setStatusLEDColor(BLACK);
    statusPixel.clear();
    statusPixel.show(); // turn off
    // TODO!
    // release for use by...
    // delete statusPixel;
  #endif

  // TODO: USE_STATUS_DOTSTAR
}


/****************************************************************************/
/*!
    @brief    Sets a status RGB LED's color
    @param    color
              Desired RGB color.
*/
/****************************************************************************/
void Wippersnapper::setStatusLEDColor(uint32_t color) {
   #ifdef USE_STATUS_NEOPIXEL
    uint8_t red = (color >> 16) & 0xff;   // red
    uint8_t green = (color >> 8) & 0xff;  // green
    uint8_t blue = color & 0xff;          // blue
    // flood all pixels
    for (int i = 0; i < STATUS_NEOPIXEL_NUM; i++) {
        statusPixel.setPixelColor(i, red, green, blue);
    }
    statusPixel.show();
  #endif

  #ifdef USE_STATUS_DOTSTAR
    uint8_t red = (color >> 16) & 0xff;   // red
    uint8_t green = (color >> 8) & 0xff;  // green
    uint8_t blue = color & 0xff;          // blue
    // flood all pixels
    for (int i = 0; i < STATUS_NEOPIXEL_NUM; i++) {
        statusPixelDotStar.setPixelColor(i, red, green, blue);
    }
    statusPixelDotStar.show();
  #endif

  // TODO: Handle non-color LED
}