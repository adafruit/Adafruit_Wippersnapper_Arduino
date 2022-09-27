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
Adafruit_NeoPixel *statusPixel = new Adafruit_NeoPixel(
    STATUS_NEOPIXEL_NUM, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

#ifdef USE_STATUS_DOTSTAR
Adafruit_DotStar *statusPixelDotStar =
    new Adafruit_DotStar(STATUS_DOTSTAR_NUM, STATUS_DOTSTAR_PIN_DATA,
                         STATUS_DOTSTAR_PIN_CLK, DOTSTAR_BRG);
#endif

/****************************************************************************/
/*!
    @brief    Initializes board-specific status LED.
    @returns  True if initialized, False if status LED hardware is already
                in-use.
*/
/****************************************************************************/
bool statusLEDInit() {
  bool is_success = false;

#ifdef USE_STATUS_NEOPIXEL
  if (!WS.lockStatusNeoPixel) {
#if defined(NEOPIXEL_I2C_POWER)
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#elif defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
    statusPixel = new Adafruit_NeoPixel(
        STATUS_NEOPIXEL_NUM, STATUS_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
    statusPixel->begin();
    statusPixel->setBrightness(10);
    statusPixel->show();
  }
#endif

#ifdef USE_STATUS_DOTSTAR
  if (WS.lockStatusDotStar == false) {
    statusPixelDotStar->begin();
    statusPixelDotStar->show(); // turn all pixels off
    statusPixelDotStar->setBrightness(5);
    WS.lockStatusDotStar = true;
    is_success = true;
  }
#endif

#ifdef USE_STATUS_LED
  pinMode(STATUS_LED_PIN, OUTPUT); // Initialize LED
  digitalWrite(STATUS_LED_PIN, 0); // Turn OFF LED
  WS.lockStatusLED = true;         // set global pin "lock" flag
  is_success = true;
#endif
  return is_success;
}

/****************************************************************************/
/*!
    @brief    De-initializes status LED. The usingStatus flag is also reset.
*/
/****************************************************************************/
void statusLEDDeinit() {
#ifdef USE_STATUS_NEOPIXEL
  statusPixel->clear();
  statusPixel->show(); // turn off
  WS.lockStatusNeoPixel = false;
#endif

#ifdef USE_STATUS_DOTSTAR
  statusPixelDotStar->clear();
  statusPixelDotStar->show(); // turn off
  WS.lockStatusDotStar = false;
#endif

#ifdef USE_STATUS_LED
  digitalWrite(STATUS_LED_PIN, 0); // turn off
  pinMode(STATUS_LED_PIN,
          INPUT);           // "release" for use by setting to input (hi-z)
  WS.lockStatusLED = false; // un-set global pin "lock" flag
#endif
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
    statusPixel->setPixelColor(i, red, green, blue);
  }
  statusPixel->show();
#endif

#ifdef USE_STATUS_DOTSTAR
  uint8_t red = (color >> 16) & 0xff;  // red
  uint8_t green = (color >> 8) & 0xff; // green
  uint8_t blue = color & 0xff;         // blue
  // flood all dotstar pixels
  for (int i = 0; i < STATUS_DOTSTAR_NUM; i++) {
    statusPixelDotStar->setPixelColor(i, green, red, blue);
  }
  statusPixelDotStar->show();
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
      statusPixel->setBrightness(i);
      statusPixel->show();
#elif defined(USE_STATUS_DOTSTAR)
      statusPixelDotStar->setBrightness(i);
      statusPixelDotStar->show();
#elif defined(ARDUINO_ARCH_ESP32) && defined(USE_STATUS_LED)
      ledcWrite(LEDC_CHANNEL_0, i);
#else
      analogWrite(STATUS_LED_PIN, i);
#endif
      delay(10);
    }

    for (int i = 200; i >= 50; i -= 5) {
#if defined(USE_STATUS_NEOPIXEL)
      statusPixel->setBrightness(i);
      statusPixel->show();
#elif defined(USE_STATUS_DOTSTAR)
      statusPixelDotStar->setBrightness(i);
      statusPixelDotStar->show();
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
    @brief    Converts the a ws_led_status_t status state to color.
    @param    statusState
              Hardware's status state.
    @return   Color as a uint32_t
*/
/****************************************************************************/
uint32_t ledStatusStateToColor(ws_led_status_t statusState) {
  uint32_t ledColor;
  switch (statusState) {
  case WS_LED_STATUS_KAT:
    ledColor = GREEN;
    break;
  case WS_LED_STATUS_ERROR_RUNTIME:
    ledColor = RED;
    break;
  case WS_LED_STATUS_WIFI_CONNECTING:
    ledColor = AMBER;
    break;
  case WS_LED_STATUS_MQTT_CONNECTING:
    ledColor = BLUE;
    break;
  case WS_LED_STATUS_WAITING_FOR_REG_MSG:
    ledColor = PINK;
    break;
  case WS_LED_STATUS_FS_WRITE:
    ledColor = YELLOW;
    break;
  default:
    ledColor = BLACK;
    break;
  }
  return ledColor;
}

/****************************************************************************/
/*!
    @brief    Sets the status LED to a specific color depending on
              the hardware's state.
    @param    statusState
              Hardware's status state.
*/
/****************************************************************************/
void statusLEDSolid(ws_led_status_t statusState = WS_LED_STATUS_ERROR_RUNTIME) {
#ifdef USE_STATUS_LED
  if (!WS.lockStatusLED)
    return;
#endif
  uint32_t ledColor = ledStatusStateToColor(statusState);
  setStatusLEDColor(ledColor);
}

/****************************************************************************/
/*!
    @brief    Blinks a status LED a specific color depending on
              the hardware's state.
    @param    statusState
              Hardware's status state.
*/
/****************************************************************************/
void statusLEDBlink(ws_led_status_t statusState) {
#ifdef USE_STATUS_LED
  if (!WS.lockStatusLED)
    return;
#endif

  // set number of times to blink
  int blinkNum = 5;
  // set blink color
  uint32_t ledColor = ledStatusStateToColor(statusState);

  while (blinkNum > 0) {
    setStatusLEDColor(ledColor);
    delay(300);
    setStatusLEDColor(BLACK);
    delay(300);
    blinkNum--;
  }
}