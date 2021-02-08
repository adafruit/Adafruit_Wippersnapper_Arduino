/*!
 * @file Wippersnapper_Checkin.cpp
 *
 * This file provides an API for registering a 
 * device with the Wippersnapper MQTT broker.
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
#include "Wippersnapper_Registration.h"

Wippersnapper_Registration::Wippersnapper_Registration(const char *machine_name, int32_t uid) {
    _machine_name = machine_name;
    _uid = uid;
}
