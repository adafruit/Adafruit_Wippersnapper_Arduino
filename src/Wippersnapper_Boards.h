/*!
 * @file Wippersnapper_Boards.h
 *
 * This file determines board type and includes network
 * interface code.
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

#ifndef ADAFRUIT_WIPPERSNAPPER_BOARDS_H
#define ADAFRUIT_WIPPERSNAPPER_BOARDS_H

#if defined(ADAFRUIT_PYPORTAL)
    #define USB_VID 0x239A
    #define USB_PID 0x8036
    #define BOARD_ID "adafruit_pyportal_m4"
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
    #define USB_VID 0x239A
    #define USB_PID 0x8038
    #define BOARD_ID "adafruit_metro_m4_airliftlite"
#elif defined(ADAFRUIT_METRO_M4_EXPRESS)
    #define USB_VID 0x239A
    #define USB_PID 0x8021
    #define BOARD_ID "adafruit_metro_m4"
#elif defined(ARDUINO_ARCH_ESP8266)
    // Feather Huzzah ESP8266, CP2104 USB/UART Bridge Controller
    #define USB_VID 0xEA60
    #define USB_PID 0x10C4
    #define BOARD_ID "adafruit_huzzah_8266"
#else
  #warning "Board not identified within Wippersnapper_Boards.h!"
#endif


#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H