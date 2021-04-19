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
    // turn sampling off for all digital pins
    for (int i = 1; i < WS.totalDigitalPins; i++) {
        _digital_input_pins[i].timerInterval = -1;
    }
}

/*********************************************************/
/*!
    @brief    Digital GPIO destructor.
*/
/*********************************************************/
Wippersnapper_DigitalGPIO::~Wippersnapper_DigitalGPIO() {
    delete _digital_input_pins;
}

/****************************************************************************/
/*!
    @brief    Configures a digital pin to behave as an input or an output.
*/
/****************************************************************************/
void Wippersnapper_DigitalGPIO::initDigitalPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, uint8_t pinName) {
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

    _digital_input_pins[pinName].pinName = pinName;
    _digital_input_pins[pinName].timerInterval = interval_ms;
}

/****************************************************************************/
/*!
    @brief    Deinitializes a previously configured digital pin.
*/
/****************************************************************************/
void Wippersnapper_DigitalGPIO::deinitDigitalPin(uint8_t pinName) {
    WS_DEBUG_PRINT("Deinitializing digital input pin ");WS_DEBUG_PRINTLN(pinName);
    char cstr[16];
    itoa(pinName, cstr, 10);
    WS_DEBUG_PRINTLN(cstr);
    pinMode(pinName, INPUT); // hi-z
}

/****************************************************************************/
/*!
    @brief    Deinitializes a digital input pin
*/
/****************************************************************************/
void Wippersnapper_DigitalGPIO::deinitDigitalInputPin(uint8_t pinName) {
    WS_DEBUG_PRINT("Freeing digital input pin D"); WS_DEBUG_PRINTLN(pinName);
    _digital_input_pins[pinName].timerInterval = -1;
}

/****************************************************************************/
/*!
    @brief    High-level digitalRead service impl. which performs a
                digitalRead.
    @returns  pinVal
                Value of pin, either HIGH or LOW
*/
/****************************************************************************/
int Wippersnapper_DigitalGPIO::digitalReadSvc(int pinName) {
    // Service using arduino `digitalRead`
    int pinVal = digitalRead(pinName);
    return pinVal;
}

/**************************************************************************/
/*!
    @brief  High-level service which outputs to a digital pin.
*/
/**************************************************************************/
void Wippersnapper_DigitalGPIO::digitalWriteSvc(uint8_t pinName, int pinValue) {
    WS_DEBUG_PRINT("Digital Pin Event: Set ");WS_DEBUG_PRINT(pinName);
    WS_DEBUG_PRINT(" to ");WS_DEBUG_PRINTLN(pinValue);
    digitalWrite(pinName, pinValue);
}

/**************************************************************************/
/*!
    @brief    Iterates thru digital inputs, checks if they
                should send data to the broker.
*/
/**************************************************************************/
void Wippersnapper_DigitalGPIO::processDigitalInputs() {
    uint32_t curTime = millis();
    // Process digital timers
    for (int i = 0; i < WS.totalDigitalPins; i++) {
        if (_digital_input_pins[i].timerInterval > -1L) { // validate if timer is enabled
            // Check if timer executes on a time period
            if (curTime - _digital_input_pins[i].timerIntervalPrv > _digital_input_pins[i].timerInterval && _digital_input_pins[i].timerInterval != 0L) {
                WS_DEBUG_PRINT("Executing periodic timer on D");WS_DEBUG_PRINTLN(_digital_input_pins[i].pinName);
                // read the pin
                int pinVal = digitalReadSvc(_digital_input_pins[i].pinName);

                // Create new signal message
                wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

                WS_DEBUG_PRINT("Encoding...")
                // Create and encode a pinEvent message
                if (!WS.encodePinEvent(&_outgoingSignalMsg, wippersnapper_pin_v1_Mode_MODE_DIGITAL, _digital_input_pins[i].pinName, pinVal)) {
                    WS_DEBUG_PRINTLN("ERROR: Unable to encode pinEvent");
                    break;
                }
                WS_DEBUG_PRINTLN("Encoded!")
                
                // Obtain size and only write out buffer to end
                size_t msgSz;
                pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields, &_outgoingSignalMsg);
                // publish event data
                WS_DEBUG_PRINT("Publishing...")
                WS._mqtt->publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
                WS_DEBUG_PRINTLN("Published!");

                // reset the timer
                _digital_input_pins[i].timerIntervalPrv = curTime;
                
            }
            // Check if timer executes on a state change
            else if (_digital_input_pins[i].timerInterval == 0L) {
                // read pin
                int pinVal = digitalReadSvc(_digital_input_pins[i].pinName);
                // only send on-change
                if (pinVal != _digital_input_pins[i].prvPinVal) {
                    WS_DEBUG_PRINT("Executing state-based timer on D");WS_DEBUG_PRINTLN(_digital_input_pins[i].pinName);

                    // Create new signal message
                    wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

                    WS_DEBUG_PRINT("Encoding pinEvent...");
                    // Create and encode a pinEvent message
                    if (!WS.encodePinEvent(&_outgoingSignalMsg, wippersnapper_pin_v1_Mode_MODE_DIGITAL, _digital_input_pins[i].pinName, pinVal)) {
                        WS_DEBUG_PRINTLN("ERROR: Unable to encode pinEvent");
                        break;
                    }
                    WS_DEBUG_PRINTLN("Encoded!");

                    // Obtain size and only write out buffer to end
                    size_t msgSz;
                    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields, &_outgoingSignalMsg);

                    // publish event data
                    WS_DEBUG_PRINT("Publishing pinEvent...");
                    WS._mqtt->publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
                    WS_DEBUG_PRINTLN("Published!");

                    // set the pin value in the timer object for comparison on next run
                    _digital_input_pins[i].prvPinVal = pinVal;

                    // reset the timer
                    _digital_input_pins[i].timerIntervalPrv = curTime;
                }
            }
        }
    }
}