/*!
 * @file BlinkaConnect_Networking.h
 *
 * This file includes network interfaces at compile-time.
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

#ifndef BLINKACONNECT_NETWORKING_H
#define BLINKACONNECT_NETWORKING_H

#if defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || \
    defined(ADAFRUIT_PYPORTAL) || defined(ADAFRUIT_METRO_M4_EXPRESS) || \
    defined(USE_AIRLIFT)
        #include "network_interfaces/BlinkaConnect_AIRLIFT.h"
        typedef BlinkaConnect_AIRLIFT BlinkaConnect_WiFi;
#else
    #warning "Must define network interface in config.h!"
#endif

#endif // BLINKACONNECT_NETWORKING_H