/*!
 * @file Wippersnapper_StatusLED.cpp
 *
 * Interfaces for the Wippersnapper status indicator LED/NeoPixel/Dotstar/RGB LED.
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 * 
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "Wippersnapper.h"

#ifdef USE_STATUS_NEOPIXEL
  Adafruit_NeoPixel *statusPixel = new Adafruit_NeoPixel(STATUS_NEOPIXEL_NUM, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

#ifdef USE_STATUS_DOTSTAR
  Adafruit_DotStar *statusPixelDotStar = new Adafruit_DotStar(STATUS_DOTSTAR_NUM, STATUS_DOTSTAR_PIN_DATA, STATUS_DOTSTAR_PIN_CLK, DOTSTAR_BRG);
#endif

/****************************************************************************/
/*!
    @brief    Initializes board-specific status LED.
*/
/****************************************************************************/
void Wippersnapper::statusLEDInit() {
    #ifdef USE_STATUS_LED
      pinMode(STATUS_LED_PIN, OUTPUT); // Initialize LED
      digitalWrite(STATUS_LED_PIN, 0); // Turn OFF LED
    #endif

    #ifdef USE_STATUS_NEOPIXEL
      statusPixel->begin();
      statusPixel->setBrightness(50);
      statusPixel->clear();
      statusPixel->show();
    #endif

    #ifdef USE_STATUS_DOTSTAR
      statusPixelDotStar->begin();
      statusPixelDotStar->setBrightness(50);
      statusPixelDotStar->clear();
      statusPixelDotStar->show();
    #endif
}

/****************************************************************************/
/*!
    @brief    De-initializes status LED. If using a NeoPixel or DotStar,
              the object is de-allocated and data/clk pins are
              set back to INPUT.
*/
/****************************************************************************/
void Wippersnapper::statusLEDDeinit() {
  #ifdef USE_STATUS_NEOPIXEL
    statusPixel->clear();
    statusPixel->show(); // turn off
    delete statusPixel;
  #endif

  #ifdef USE_STATUS_DOTSTAR
    statusPixelDotStar->clear();
    statusPixelDotStar->show(); // turn off
    delete statusPixelDotStar
  #endif

  #ifdef USE_STATUS_LED
    digitalWrite(STATUS_LED_PIN, 0); // turn off
    pinMode(STATUS_LED_PIN, INPUT);             // "release" for use by setting to input (hi-z)
  #endif
}

/****************************************************************************/
/*!
    @brief    Sets a status RGB LED's color
    @param    color
              Desired RGB color.
*/
/****************************************************************************/
void Wippersnapper::setStatusLEDColor(uint32_t color) {
    // unpack color into (r, g, b)
   #ifdef USE_STATUS_NEOPIXEL || USE_STATUS_DOTSTAR
    uint8_t red = (color >> 16) & 0xff;   // red
    uint8_t green = (color >> 8) & 0xff;  // green
    uint8_t blue = color & 0xff;          // blue
  #endif

  #ifdef USE_STATUS_NEOPIXEL
    // flood all neopixels
    for (int i = 0; i < STATUS_NEOPIXEL_NUM; i++) {
        statusPixel->setPixelColor(i, red, green, blue);
    }
    statusPixel->show();
  #endif

  #ifdef USE_STATUS_DOTSTAR
    // flood all dotstar pixels
    for (int i = 0; i < STATUS_NEOPIXEL_NUM; i++) {
        statusPixelDotStar.setPixelColor(i, red, green, blue);
    }
    statusPixelDotStar.show();
  #endif

  #ifdef USE_STATUS_LED
    // via https://github.com/adafruit/circuitpython/blob/main/supervisor/shared/status_leds.c
    digitalWrite(STATUS_LED_PIN, color > 0);
  #endif
}

/****************************************************************************/
/*!
    @brief    Blinks a status LED a specific color depending on
              the hardware's state.
    @param    statusState
              Hardware's status state.
*/
/****************************************************************************/
void Wippersnapper::statusLEDBlink(ws_led_status_t statusState) {
    int blinkNum = 0;
    uint32_t ledBlinkColor;
    switch(statusState) {
        case WS_LED_STATUS_CONNECTED:
            blinkNum = 3;
            ledBlinkColor = LED_CONNECTED;
            break;
        case WS_LED_STATUS_ERROR:
            blinkNum = 2;
            ledBlinkColor = LED_ERROR;
        default:
            break;
    }

    while (blinkNum > 0) {
        setStatusLEDColor(ledBlinkColor);
        delay(250);
        setStatusLEDColor(BLACK);
        delay(250);
        blinkNum--;
    }
}