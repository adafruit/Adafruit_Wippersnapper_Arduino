/*!
 * @file Wippersnapper_Checkin.cpp
 *
 * This file provides an API for interacting with
 * a board's digital GPIO pins.
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

#include "Wippersnapper_DigitalGPIO.h"

Wippersnapper_DigitalGPIO::Wippersnapper_DigitalGPIO(int32_t totalDigitalGPIOPins) {
    _digital_input_pins = new digitalInputPin[totalDigitalGPIOPins];
    // turn off all digital pins
    for (int i = 1; i < WS.totalDigitalPins; i++) {
        _digital_input_pins[i].timerInterval = -1;
    }
}

Wippersnapper_DigitalGPIO::~Wippersnapper_DigitalGPIO() {
    delete _digital_input_pins;
}

/****************************************************************************/
/*!
    @brief    Configures a digital pin to behave as an input or an output.
*/
/****************************************************************************/
void initDigitalPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, uint8_t pinName) {
     if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
        WS_DEBUG_PRINT("Configured digital output pin on D"); WS_DEBUG_PRINTLN(pinName);
        pinMode(pinName, OUTPUT);
        digitalWrite(pinName, LOW); // initialize LOW
     }
     else if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS_DEBUG_PRINT("Configuring digital input pin on D"); WS_DEBUG_PRINTLN(pinName);
        pinMode(pinName, INPUT);
     }
     else {
         WS_DEBUG_PRINTLN("ERROR: Invalid digital pin direction!");
     }
}

/****************************************************************************/
/*!
    @brief    Initializes a digital input pin
*/
/****************************************************************************/
void Wippersnapper_DigitalGPIO::initDigitalInputPin(int pinName, float interval) {
    WS_DEBUG_PRINT("Init. digital input pin D");WS_DEBUG_PRINTLN(pinName);
    // Interval is in seconds, cast it to long and convert it to milliseconds
    long interval_ms = (long)interval * 1000;
    WS_DEBUG_PRINT("Interval (ms):"); WS_DEBUG_PRINTLN(interval_ms);

    WS._digital_input_pins[pinName].pinName = pinName;
    WS._digital_input_pins[pinName].timerInterval = interval_ms;
}

/****************************************************************************/
/*!
    @brief    Deinitializes a digital input pin
*/
/****************************************************************************/
void Wippersnapper::deinitDigitalInputPin(uint8_t pinName) {
    WS_DEBUG_PRINT("Freeing digital input pin D"); WS_DEBUG_PRINTLN(pinName);
    WS._digital_input_pins[pinName].timerInterval = -1;
}

/****************************************************************************/
/*!
    @brief    High-level digitalRead service impl. which performs a
                digitalRead.
    @returns  pinVal
                Value of pin, either HIGH or LOW
*/
/****************************************************************************/
int digitalReadSvc(int pinName) {
    // Service using arduino `digitalRead`
    int pinVal = digitalRead(pinName);
    return pinVal;
}

/**************************************************************************/
/*!
    @brief  High-level service which outputs to a digital pin.
*/
/**************************************************************************/
void digitalWriteSvc(uint8_t pinName, int pinValue) {
    WS_DEBUG_PRINT("Digital Pin Event: Set ");WS_DEBUG_PRINT(pinName);
    WS_DEBUG_PRINT(" to ");WS_DEBUG_PRINTLN(pinValue);
    digitalWrite(pinName, pinValue);
}
