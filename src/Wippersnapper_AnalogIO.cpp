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
    _totalAnalogInputPins = totalAnalogInputPins;

    // allocate analog input pins
    _analog_input_pins = new analogInputPin[_totalAnalogInputPins];
    for (int pin = 0; pin < _totalAnalogInputPins; pin++) {
        // turn sampling off
        _analog_input_pins[pin].period = -1;
    }
}

/***********************************************************************************/
/*!
    @brief  Destructor for Analog IO class.
*/
/***********************************************************************************/
Wippersnapper_AnalogIO::~Wippersnapper_AnalogIO() {
    _vRef = 0.0;
    delete _analog_input_pins;
}

/***********************************************************************************/
/*!
    @brief  Initializes an analog pin
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin, float period, wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode) {
    if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
        WS_DEBUG_PRINTLN("ERROR: Analog output pin not yet implemented");
    } else if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        initAnalogInputPin(pin, period, analogReadMode);
    } else {
        WS_DEBUG_PRINTLN("ERROR: Unable to decode analog pin direction");
    }
}


/***********************************************************************************/
/*!
    @brief  Initializes an analog input pin
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogInputPin(int pin, float period, wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode) {
        WS_DEBUG_PRINT("Initialized analog input pin A");WS_DEBUG_PRINTLN(pin);
        // Set pin mode
        // TODO: Support INPUT_PULLUP
        pinMode(pin, INPUT);

        // Period is in seconds, cast it to long and convert it to milliseconds
        long periodMs = (long)period * 1000;
        WS_DEBUG_PRINT("Interval (ms):"); WS_DEBUG_PRINTLN(periodMs);

        // Configure analog pin object
        _analog_input_pins[pin].pinName = pin;
        _analog_input_pins[pin].period = periodMs;
        _analog_input_pins[pin].readMode = analogReadMode;
}

void Wippersnapper_AnalogIO::deinitAnalogInputPinObj(int pin) {
    _analog_input_pins[pin].pinName = NULL;
    _analog_input_pins[pin].readMode = wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_UNSPECIFIED;
    _analog_input_pins[pin].period = NULL;
    _analog_input_pins[pin].prvPinVal = NULL;
    _analog_input_pins[pin].prvPeriod = NULL;
}

/***********************************************************************************/
/*!
    @brief  Deinitializes an analog pin.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::deinitAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin) {
    WS_DEBUG_PRINT("Deinitializing analog pin"); WS_DEBUG_PRINTLN(pin);
    if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS_DEBUG_PRINTLN("Deinitialized analog input pin obj.");
       deinitAnalogInputPinObj(pin);
    }
    char cstr[16];
    itoa(pin, cstr, 10);
    WS_DEBUG_PRINTLN(cstr);
    pinMode(pin, INPUT); // hi-z

}