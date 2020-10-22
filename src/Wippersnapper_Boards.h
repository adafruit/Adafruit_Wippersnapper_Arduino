/*!
 * @file Wippersnapper_Boards.h
 *
 * This file determines board type at compile-time
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
#else
  #warning "Board not identified within Wippersnapper_Boards.h!"
#endif


#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H