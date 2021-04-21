/*!
 * @file Wippersnapper_AnalogIO.h
 *
 * This file provides digital GPIO control and access.
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

#ifndef WIPPERSNAPPER_ANALOGIO_H
#define WIPPERSNAPPER_ANALOGIO_H

#include "Wippersnapper.h"

// Holds data about an analog input pin
// TODO

// forward decl.
class Wippersnapper;

class Wippersnapper_AnalogIO {
    public:
        Wippersnapper_AnalogIO(int32_t totalAnalogInputPins);
        ~Wippersnapper_AnalogIO();


        // TODO!
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DIGITALGPIO_H