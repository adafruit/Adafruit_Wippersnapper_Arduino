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
struct analogInputPin {
    int pinName;  // Pin name
    long period;      // Pin timer interval, in millis, -1 if disabled.
    long prvPeriod;   // When Pin's timer was previously serviced, in millis
    float prvPinVal;    // Previous pin value
};

// forward decl.
class Wippersnapper;

class Wippersnapper_AnalogIO {
    public:
        Wippersnapper_AnalogIO(int32_t totalAnalogInputPins, float vref);
        ~Wippersnapper_AnalogIO();

        void initAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin, float period, wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode);
        void deinitAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin);

    private:
        float _vRef;                   /*!< Hardware's default voltage reference */
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DIGITALGPIO_H