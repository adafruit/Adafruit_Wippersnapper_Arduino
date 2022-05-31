/*!
 * @file Wippersnapper_StatusLED.cpp
 *
 * Interfaces for the Wippersnapper status indicator LED/NeoPixel/Dotstar/RGB
 * LED.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "Wippersnapper_StatusLED.h"
#include "Wippersnapper.h"

extern Wippersnapper WS;

#ifdef USE_STATUS_NEOPIXEL
Adafruit_NeoPixel *_statusPixelNeo;
#endif

#ifdef USE_STATUS_DOTSTAR
Adafruit_DotStar *_statusPixelDotStar;
#endif

/****************************************************************************/
/*!
    @brief    Initializes board-specific status LED.
*/
/****************************************************************************/
void statusLEDInit() {

// init. monocolor status LED
#ifdef USE_STATUS_LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, 0);
#endif

// TODO: NeoPixel and DotStar should first be checked
// if they're being used as a component, and de-init'd
// init. NeoPixel to use as a status LED
#ifdef USE_STATUS_NEOPIXEL
    #if defined(NEOPIXEL_I2C_POWER)
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    #elif defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, NEOPIXEL_POWER_ON);
    #endif
    _statusPixelNeo = new Adafruit_NeoPixel(STATUS_NEOPIXEL_NUM, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
    _statusPixelNeo->begin();
    _statusPixelNeo->setBrightness(10);
    _statusPixelNeo->show();
#endif

// init. DotStar to use as a status LED
#ifdef USE_STATUS_DOTSTAR
    _statusPixelDotStar = new Adafruit_DotStar(STATUS_DOTSTAR_NUM, STATUS_DOTSTAR_PIN_DATA, STATUS_DOTSTAR_PIN_CLK, DOTSTAR_BRG);
    _statusPixelDotStar->begin();
    _statusPixelDotStar->setBrightness(5);
    _statusPixelDotStar->show(); // turn all pixels off
#endif

 WS.statusLEDActive = true;
}

/****************************************************************************/
/*!
    @brief    De-initializes the status LED.
*/
/****************************************************************************/
void statusLEDDeinit() {
// de-init NeoPixel
#ifdef USE_STATUS_NEOPIXEL
  _statusPixelNeo->clear();
  _statusPixelNeo->show(); // turn off
  delete _statusPixelNeo;
#endif

// de-init DotStar
#ifdef USE_STATUS_DOTSTAR
  _statusPixelDotStar->clear();
  _statusPixelDotStar->show(); // turn off
  delete _statusPixelDotStar;
#endif

#ifdef USE_STATUS_LED
  digitalWrite(STATUS_LED_PIN, 0); // turn off
  pinMode(STATUS_LED_PIN,
          INPUT);           // "release" for use by setting to input (hi-z)
#endif
 WS.statusLEDActive = false;
}

/****************************************************************************/
/*!
    @brief    Sets a status RGB LED's color
    @param    color
              Desired RGB color.
*/
/****************************************************************************/
void setStatusLEDColor(uint32_t color) {
#ifdef USE_STATUS_NEOPIXEL
  uint8_t red = (color >> 16) & 0xff;  // red
  uint8_t green = (color >> 8) & 0xff; // green
  uint8_t blue = color & 0xff;         // blue
  // flood all neopixels
  for (int i = 0; i < STATUS_NEOPIXEL_NUM; i++) {
    _statusPixelNeo->setPixelColor(i, red, green, blue);
  }
  _statusPixelNeo->show();
#endif

#ifdef USE_STATUS_DOTSTAR
  uint8_t red = (color >> 16) & 0xff;  // red
  uint8_t green = (color >> 8) & 0xff; // green
  uint8_t blue = color & 0xff;         // blue
  // flood all dotstar pixels
  for (int i = 0; i < STATUS_DOTSTAR_NUM; i++) {
    _statusPixelDotStar->setPixelColor(i, green, red, blue);
  }
  _statusPixelDotStar->show();
#endif

#ifdef USE_STATUS_LED
  // via
  // https://github.com/adafruit/circuitpython/blob/main/supervisor/shared/status_leds.c
  digitalWrite(STATUS_LED_PIN, color > 0);
#endif
}

/****************************************************************************/
/*!
    @brief    Fades the status LED.
    @param    color
              The specific color to fade the status LED.
    @param    numFades
              The amount of time to fade/pulse the status LED.
*/
/****************************************************************************/
void statusLEDFade(uint32_t color, int numFades = 3) {
  setStatusLEDColor(color);

// attach LEDC pin
#if defined(ARDUINO_ARCH_ESP32) && defined(USE_STATUS_LED)
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(STATUS_LED_PIN, LEDC_CHANNEL_0);
#endif

  // pulse numFades times
  for (int i = 0; i < numFades; i++) {
    for (int i = 50; i <= 200; i += 5) {
#if defined(USE_STATUS_NEOPIXEL)
      _statusPixelNeo->setBrightness(i);
      _statusPixelNeo->show();
#elif defined(USE_STATUS_DOTSTAR)
      _statusPixelDotStar->setBrightness(i);
      _statusPixelDotStar->show();
#elif defined(ARDUINO_ARCH_ESP32) && defined(USE_STATUS_LED)
      ledcWrite(LEDC_CHANNEL_0, i);
#else
      analogWrite(STATUS_LED_PIN, i);
#endif
      delay(10);
    }

    for (int i = 200; i >= 50; i -= 5) {
#if defined(USE_STATUS_NEOPIXEL)
      _statusPixelNeo->setBrightness(i);
      _statusPixelNeo->show();
#elif defined(USE_STATUS_DOTSTAR)
      _statusPixelDotStar->setBrightness(i);
      _statusPixelDotStar->show();
#elif defined(ARDUINO_ARCH_ESP32) && defined(USE_STATUS_LED)
      ledcWrite(LEDC_CHANNEL_0, i);
#else
      analogWrite(STATUS_LED_PIN, i);
#endif
      delay(10);
    }
  }

// detach LEDC pin and reset its pinMode
#if defined(ARDUINO_ARCH_ESP32) && defined(USE_STATUS_LED)
  ledcDetachPin(STATUS_LED_PIN);
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif

  // clear status LED color
  setStatusLEDColor(BLACK);
}

/****************************************************************************/
/*!
    @brief    Blinks a status LED a specific color depending on
              the hardware's state.
    @param    statusState
              Hardware's status state.
    @param    blinkFast
              Blink the LED for 100ms instead of default 300ms.
*/
/****************************************************************************/
void statusLEDBlink(ws_led_status_t statusState, bool blinkFast) {
  int blinkNum;
  uint32_t ledBlinkColor;

  // are we going to blink slowly (connecting) or quickly (error)?
  long delayTime = 300;
  if (blinkFast)
    delayTime = 100;

  // what color are we goign to blink?
  switch (statusState) {
  case WS_LED_STATUS_KAT:
    ledBlinkColor = GREEN;
    break;
  case WS_LED_STATUS_ERROR_RUNTIME:
    ledBlinkColor = RED;
    break;
  case WS_LED_STATUS_WIFI_CONNECTING:
    blinkNum = 4; // delay(1200)
    ledBlinkColor = AMBER;
    break;
  case WS_LED_STATUS_MQTT_CONNECTING:
    blinkNum = 4; // delay(1200)
    ledBlinkColor = BLUE;
    break;
  case WS_LED_STATUS_WAITING_FOR_REG_MSG:
    ledBlinkColor = PINK;
    break;
  case WS_LED_STATUS_FS_WRITE:
    ledBlinkColor = YELLOW;
    break;
  default:
    ledBlinkColor = BLACK;
    break;
  }

  // blink!
  blinkNum = 3;
  while (blinkNum > 0) {
    setStatusLEDColor(ledBlinkColor);
    delay(delayTime);
    setStatusLEDColor(BLACK);
    delay(delayTime);
    blinkNum--;
  }
}