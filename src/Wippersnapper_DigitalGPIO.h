/*!
 * @file Wippersnapper_Checkin.h
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

#ifndef WIPPERSNAPPER_DIGITALGPIO_H
#define WIPPERSNAPPER_DIGITALGPIO_H

#include "Wippersnapper.h"

// Holds data about a digital input pin
struct digitalInputPin {
    uint8_t pinName;       // Pin name
    long timerInterval;    // Timer interval, in millis, -1 if disabled.
    long timerIntervalPrv; // When timer was previously serviced, in millis
    int prvPinVal;         // Previous pin value
};

// forward decl.
class Wippersnapper;

class Wippersnapper_DigitalGPIO {
    public:
        Wippersnapper_DigitalGPIO(int32_t totalDigitalGPIOPins);
        ~Wippersnapper_DigitalGPIO();
        void initDigitalInputPin(int pinName, float interval);
        void deinitDigitalInputPin(uint8_t pinName);

        digitalInputPin* _digital_input_pins; /*!< Array of gpio pin objects */
    private:
    protected:
};

extern Wippersnapper WS;
#endif // WIPPERSNAPPER_DIGITALGPIO_H