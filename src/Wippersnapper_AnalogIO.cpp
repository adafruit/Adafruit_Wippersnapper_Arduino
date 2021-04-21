/*!
 * @file Wippersnapper_AnalogIO.cpp
 *
 * This file provides an API for interacting with
 * a board's analog IO pins.
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

#include "Wippersnapper_AnalogIO.h"

/***********************************************************************************/
/*!
    @brief  Initializes Analog IO class.
    @param  totalAnalogInputPins
                Total number of analog input pins to allocate.
*/
/***********************************************************************************/
Wippersnapper_AnalogIO::Wippersnapper_AnalogIO(int32_t totalAnalogInputPins, float vRef) {
    _vRef = vRef;
}

/***********************************************************************************/
/*!
    @brief  Destructor for Analog IO class.
*/
/***********************************************************************************/
Wippersnapper_AnalogIO::~Wippersnapper_AnalogIO() {
    // TODO
}

/***********************************************************************************/
/*!
    @brief  Initializes an analog pin
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin, float period, wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode) {
    if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
        WS_DEBUG_PRINTLN("ERROR: Analog output pin not implemented yet");
    } else if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS_DEBUG_PRINTLN("Analog input pin");
    } else {
        WS_DEBUG_PRINTLN("ERROR: Unable to decode analog pin direction");
    }
}

/***********************************************************************************/
/*!
    @brief  Deinitializes an analog pin
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::deinitAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin) {
    WS_DEBUG_PRINT("Deinitializing analog pin"); WS_DEBUG_PRINTLN(pin);
    if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS_DEBUG_PRINTLN("Deinitialized analog input pin obj.");
        // TODO: set timer period to -1
    }
    char cstr[16];
    itoa(pin, cstr, 10);
    WS_DEBUG_PRINTLN(cstr);
    pinMode(pin, INPUT); // hi-z
}