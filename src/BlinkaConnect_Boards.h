/*!
 * @file WipperSnapper_Boards.h
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
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
    #define USB_VID 0x239A
    #define USB_PID 0x8038
#elif defined(ADAFRUIT_METRO_M4_EXPRESS)
    #define USB_VID 0x239A
    #define USB_PID 0x8021
#else
  #warning "Board not identified within WipperSnapper_Boards.h!"
#endif


#endif // ADAFRUIT_WIPPERSNAPPER_BOARDS_H