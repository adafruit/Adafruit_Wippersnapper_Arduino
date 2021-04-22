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
    @brief  Initializes an analog output pin.
    @param  pin
                The analog pin to read from.
    @param  direction 
                The pin's direction.
    @param  period
                Time between measurements, in seconds.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogPin(int pin) {
    WS_DEBUG_PRINTLN("ERROR: Analog output pin not yet implemented");
}

/***********************************************************************************/
/*!
    @brief  Initializes an analog input pin
    @param  pin
                The analog pin to read from.
    @param  period
                Time between measurements, in seconds.
    @param  analogReadMode
                The type of measurement to perform. Returns either a raw ADC value
                    or the pin's voltage (in volts).
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogInputPin(int pin, float period, wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode) {
        WS_DEBUG_PRINT("Initialized analog input pin A");WS_DEBUG_PRINTLN(pin);
        // Set pin mode
        // TODO: Support INPUT_PULLUP pull
        pinMode(pin, INPUT);

        // Period is in seconds, cast it to long and convert it to milliseconds
        long periodMs = (long)period * 1000;
        WS_DEBUG_PRINT("Interval (ms):"); WS_DEBUG_PRINTLN(periodMs);

        // Configure analog pin object
        _analog_input_pins[pin].pinName = pin;
        _analog_input_pins[pin].period = periodMs;
        _analog_input_pins[pin].readMode = analogReadMode;
}

/***********************************************************************************/
/*!
    @brief  Deinitializes an analog input pin.
    @param  pin
                The analog input pin to deinitialize.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::deinitAnalogInputPinObj(int pin) {
    _analog_input_pins[pin].pinName = NULL;
    _analog_input_pins[pin].readMode = wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_UNSPECIFIED;
    _analog_input_pins[pin].period = -1;
    _analog_input_pins[pin].prvPinVal = NULL;
    _analog_input_pins[pin].prvPeriod = NULL;
}

/***********************************************************************************/
/*!
    @brief  Deinitializes an analog pin.
    @param  direction
                The analog pin's direction.
    @param  pin
                The analog pin to deinitialize.

*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::deinitAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin) {
    WS_DEBUG_PRINT("Deinitializing analog pin A"); WS_DEBUG_PRINTLN(pin);
    if (direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS_DEBUG_PRINTLN("Deinitialized analog input pin obj.");
       deinitAnalogInputPinObj(pin);
    }
    pinMode(pin, INPUT); // hi-z
}

/**********************************************************/
/*!
    @brief    Reads the value of an analog pin.
                Value is always scaled to 16-bit.
*/
/**********************************************************/
uint16_t Wippersnapper_AnalogIO::readAnalogPinRaw(int pin) {
    uint16_t value;
    value = analogRead(pin);
    // lshift for 16bit res.
    value = value << 4;
    return value;
}

/**********************************************************/
/*!
    @brief    Calculates analog pin's voltage provided
               a 16-bit ADC value.
*/
/**********************************************************/
float Wippersnapper_AnalogIO::getAnalogPinVoltage(uint16_t rawValue) {
    float pinVoltage;
    pinVoltage = rawValue * _vRef / 65536;
    return pinVoltage;
}

/**********************************************************/
/*!
    @brief    Encodes an analog input pin event into a
                signal message.
    @param    outgoingSignalMsg
                Pointer to an empty CreateSignalRequest
                message.
    @param    pinName
                Specifies the pin's name.
    @param    pinVal
                Pin value.
*/
/**********************************************************/
bool Wippersnapper_AnalogIO::encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg, uint8_t pinName, uint16_t pinVal) {
    bool is_success = true;
    WS_DEBUG_PRINT("Encoding signal message...");
    // fill the pin_event message
    outgoingSignalMsg->which_payload = wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
    outgoingSignalMsg->payload.pin_event.mode = wippersnapper_pin_v1_Mode_MODE_ANALOG;
    sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "A%d", pinName);
    sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%u", pinVal);

    // Encode signal message
    pb_ostream_t stream = pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
    if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields, outgoingSignalMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
        is_success = false;
    }

    WS_DEBUG_PRINTLN("Encoded!");
    return is_success;
}

/**********************************************************/
/*!
    @brief    Encodes an analog input pin event into a
                signal message.
    @param    outgoingSignalMsg
                Pointer to an empty CreateSignalRequest
                message.
    @param    pinName
                Specifies the pin's name.
    @param    pinVal
                Pin voltage, in volts.
*/
/**********************************************************/
bool Wippersnapper_AnalogIO::encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg, uint8_t pinName, float pinVal) {
    bool is_success = true;

    // fill the pin_event message
    outgoingSignalMsg->which_payload = wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
    outgoingSignalMsg->payload.pin_event.mode = wippersnapper_pin_v1_Mode_MODE_ANALOG;
    sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "A%d", pinName);
    sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%f", pinVal);

    // Encode signal message
    pb_ostream_t stream = pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
    if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields, outgoingSignalMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
        is_success = false;
    }

    return is_success;
}

/**********************************************************/
/*!
    @brief    Iterates thru analog inputs
*/
/**********************************************************/
void Wippersnapper_AnalogIO::processAnalogInputs() {
    uint16_t pinValue;
    float pinVoltage;

    uint32_t curTime = millis();

    // Process analog input pins
    for (int i = 0; i < _totalAnalogInputPins; i++) {
        if (_analog_input_pins[i].period > -1L) { // validate if pin is enabled for sampling
            // pin executes on-period
            if (curTime - _analog_input_pins[i].prvPeriod > _analog_input_pins[i].period && _analog_input_pins[i].period != 0L) {
                WS_DEBUG_PRINT("Executing periodic event on A");WS_DEBUG_PRINTLN(_analog_input_pins[i].pinName);
                // Create new signal message
                wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;
                // Perform an analog read
                pinValue = readAnalogPinRaw(_analog_input_pins[i].pinName);
                // Determine if we should convert reading to voltage
                if (_analog_input_pins[i].readMode == wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
                    pinVoltage = getAnalogPinVoltage(pinValue);
                }

                // encode the signal message
                if (_analog_input_pins[i].readMode == wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
                    if (!encodePinEvent(&_outgoingSignalMsg, _analog_input_pins[i].pinName, pinVoltage)) {
                        WS_DEBUG_PRINTLN("ERROR: Unable to encode pinevent (analog input, voltage");
                    }
                }
                else {
                    if (!encodePinEvent(&_outgoingSignalMsg, _analog_input_pins[i].pinName, pinValue)) {
                        WS_DEBUG_PRINTLN("ERROR: Unable to encode pinevent (analog input, value");
                    }
                }

                // Obtain size and only write out buffer to end
                size_t msgSz;
                pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields, &_outgoingSignalMsg);
                // publish event data
                WS_DEBUG_PRINT("Publishing...")
                WS._mqtt->publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
                WS_DEBUG_PRINTLN("Published!");

                // reset the digital pin
                _analog_input_pins[i].prvPeriod = curTime;
            }
            // pin sample on-change
            else if (_analog_input_pins[i].period == 0L) {
                // Perform an analog read
                pinValue = readAnalogPinRaw(_analog_input_pins[i].pinName);
                if (pinValue != _analog_input_pins[i].prvPinVal) {
                    WS_DEBUG_PRINT("Executing state-based event on A");WS_DEBUG_PRINTLN(_analog_input_pins[i].pinName);

                    // Create new signal message
                    wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

                    // Determine if we should convert reading to voltage
                    if (_analog_input_pins[i].readMode == wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
                        pinVoltage = getAnalogPinVoltage(pinValue);
                    }

                    // encode the signal message
                    if (_analog_input_pins[i].readMode == wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
                        if (!encodePinEvent(&_outgoingSignalMsg, _analog_input_pins[i].pinName, pinVoltage)) {
                            WS_DEBUG_PRINTLN("ERROR: Unable to encode pinevent (analog input, voltage");
                        }
                    }
                    else {
                        if (!encodePinEvent(&_outgoingSignalMsg, _analog_input_pins[i].pinName, pinValue)) {
                            WS_DEBUG_PRINTLN("ERROR: Unable to encode pinevent (analog input, value");
                        }
                    }

                    // Obtain msg size and only write out buffer to end
                    size_t msgSz;
                    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields, &_outgoingSignalMsg);
                    // publish event data
                    WS_DEBUG_PRINT("Publishing...")
                    WS._mqtt->publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
                    WS_DEBUG_PRINTLN("Published!");

                    // set the pin value in the digital pin object for comparison on next run
                    _analog_input_pins[i].prvPinVal = pinValue;

                    // reset the digital pin
                    _analog_input_pins[i].prvPeriod = curTime;
                }
            }
        }
    }
}