/*!
 * @file Wippersnapper_DigitalGPIO.cpp
 *
 * This file provides an API for interacting with
 * a board's digital GPIO pins.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper_DigitalGPIO.h"

/***********************************************************************************/
/*!
    @brief  Initializes DigitalGPIO class.
    @param  totalDigitalInputPins
                Total number of digital gpio input-capable pins to allocate.
*/
/***********************************************************************************/
Wippersnapper_DigitalGPIO::Wippersnapper_DigitalGPIO(
    int32_t totalDigitalInputPins) {
  _totalDigitalInputPins = totalDigitalInputPins;
  _digital_input_pins = new digitalInputPin[_totalDigitalInputPins];
  // turn input sampling off for all digital pins
  for (int i = 0; i < _totalDigitalInputPins; i++) {
    _digital_input_pins[i].pinName = -1;
    _digital_input_pins[i].period = -1;
    _digital_input_pins[i].prvPeriod = 0L;
    _digital_input_pins[i].prvPinVal = 0;
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

/*******************************************************************************************************************************/
/*!
    @brief  Configures a digital pin to behave as an input or an output.
    @param  direction
            The pin's direction.
    @param  pinName
            The pin's name.
    @param  period
            The pin's period, in seconds.
    @param  pull
            The pin's pull mode.
*/
/*******************************************************************************************************************************/
void Wippersnapper_DigitalGPIO::initDigitalPin(
    wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
    uint8_t pinName, float period,
    wippersnapper_pin_v1_ConfigurePinRequest_Pull pull) {
  if (direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {

#ifdef STATUS_LED_PIN
    // deinit status led, use it as a dio component instead
    if (pinName == STATUS_LED_PIN)
      statusLEDDeinit();
#endif
    WS_DEBUG_PRINT("Configured digital output pin on D");
    WS_DEBUG_PRINTLN(pinName);
    pinMode(pinName, OUTPUT);
    digitalWrite(pinName, LOW); // initialize LOW
  } else if (
      direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
    WS_DEBUG_PRINT("Configuring digital input pin on D");
    WS_DEBUG_PRINT(pinName);
    if (pull == wippersnapper_pin_v1_ConfigurePinRequest_Pull_PULL_UP) {
      WS_DEBUG_PRINTLN("with internal pull-up enabled");
      pinMode(pinName, INPUT_PULLUP);
    } else {
      pinMode(pinName, INPUT);
      WS_DEBUG_PRINT("\n");
    }

    // Period is in seconds, cast it to long and convert it to milliseconds
    long periodMs = (long)period * 1000;
    WS_DEBUG_PRINT("Interval (ms):");
    WS_DEBUG_PRINTLN(periodMs);

    // attempt to allocate a pinName within _digital_input_pins[]
    for (int i = 0; i < _totalDigitalInputPins; i++) {
      if (_digital_input_pins[i].period == -1L) {
        _digital_input_pins[i].pinName = pinName;
        _digital_input_pins[i].period = periodMs;
        break;
      }
    }

  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid digital pin direction!");
  }
}

/********************************************************************************************************************************/
/*!
    @brief    Deinitializes a previously configured digital pin.
    @param    direction
              The pin's direction.
    @param    pinName
              The pin's name.
*/
/********************************************************************************************************************************/
void Wippersnapper_DigitalGPIO::deinitDigitalPin(
    wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
    uint8_t pinName) {
  WS_DEBUG_PRINT("Deinitializing digital pin ");
  WS_DEBUG_PRINTLN(pinName);
  if (direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
    // de-allocate the pin within digital_input_pins[]
    for (int i = 0; i < _totalDigitalInputPins; i++) {
      if (_digital_input_pins[i].pinName == pinName) {
        _digital_input_pins[i].pinName = -1;
        _digital_input_pins[i].period = -1;
        _digital_input_pins[i].prvPeriod = 0L;
        _digital_input_pins[i].prvPinVal = 0;
        break;
      }
    }
  }
  char cstr[16];
  itoa(pinName, cstr, 10);
  pinMode(pinName, INPUT); // hi-z

// if prv. in-use by DIO, release pin back to application
#ifdef STATUS_LED_PIN
  if (pinName == STATUS_LED_PIN)
    statusLEDInit();
#endif
}

/********************************************************************/
/*!
    @brief    High-level digitalRead service impl. which performs a
                digitalRead.
    @param    pinName
                The pin's name
    @returns  The pin's value.
*/
/********************************************************************/
int Wippersnapper_DigitalGPIO::digitalReadSvc(int pinName) {
  // Service using arduino `digitalRead`
  int pinVal = digitalRead(pinName);
  return pinVal;
}

/*******************************************************************************/
/*!
    @brief  Writes a value to a pin.
    @param  pinName
                The pin's name.
    @param  pinValue
                The pin's value.
*/
/*******************************************************************************/
void Wippersnapper_DigitalGPIO::digitalWriteSvc(uint8_t pinName, int pinValue) {
  WS_DEBUG_PRINT("Digital Pin Event: Set ");
  WS_DEBUG_PRINT(pinName);
  WS_DEBUG_PRINT(" to ");
  WS_DEBUG_PRINTLN(pinValue);
  digitalWrite(pinName, pinValue);
}

/**********************************************************/
/*!
    @brief    Iterates thru digital inputs, checks if they
                should send data to the broker.
*/
/**********************************************************/
void Wippersnapper_DigitalGPIO::processDigitalInputs() {
  long curTime = millis();
  // Process digital digital pins
  for (int i = 0; i < _totalDigitalInputPins; i++) {
    if (_digital_input_pins[i].period >
        -1L) { // validate if digital pin is enabled
      // Check if digital pin executes on a time period
      if (curTime - _digital_input_pins[i].prvPeriod >
              _digital_input_pins[i].period &&
          _digital_input_pins[i].period != 0L) {
        WS_DEBUG_PRINT("Executing periodic event on D");
        WS_DEBUG_PRINTLN(_digital_input_pins[i].pinName);
        // read the pin
        int pinVal = digitalReadSvc(_digital_input_pins[i].pinName);

        // Create new signal message
        wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg =
            wippersnapper_signal_v1_CreateSignalRequest_init_zero;

        WS_DEBUG_PRINT("Encoding...")
        // Create and encode a pinEvent message
        if (!WS.encodePinEvent(&_outgoingSignalMsg,
                               _digital_input_pins[i].pinName, pinVal)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to encode pinEvent");
          break;
        }
        WS_DEBUG_PRINTLN("Encoded!")

        // Obtain size and only write out buffer to end
        size_t msgSz;
        pb_get_encoded_size(&msgSz,
                            wippersnapper_signal_v1_CreateSignalRequest_fields,
                            &_outgoingSignalMsg);

        WS_DEBUG_PRINT("Publishing pinEvent...");
        WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
        WS_DEBUG_PRINTLN("Published!");

        // reset the digital pin
        _digital_input_pins[i].prvPeriod = curTime;
      } else if (_digital_input_pins[i].period == 0L) {
        // read pin
        int pinVal = digitalReadSvc(_digital_input_pins[i].pinName);
        // only send on-change
        if (pinVal != _digital_input_pins[i].prvPinVal) {
          WS_DEBUG_PRINT("Executing state-based event on D");
          WS_DEBUG_PRINTLN(_digital_input_pins[i].pinName);

          // Create new signal message
          wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg =
              wippersnapper_signal_v1_CreateSignalRequest_init_zero;

          WS_DEBUG_PRINT("Encoding pinEvent...");
          // Create and encode a pinEvent message
          if (!WS.encodePinEvent(&_outgoingSignalMsg,
                                 _digital_input_pins[i].pinName, pinVal)) {
            WS_DEBUG_PRINTLN("ERROR: Unable to encode pinEvent");
            break;
          }
          WS_DEBUG_PRINTLN("Encoded!");

          // Obtain size and only write out buffer to end
          size_t msgSz;
          pb_get_encoded_size(
              &msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields,
              &_outgoingSignalMsg);
          WS_DEBUG_PRINT("Publishing pinEvent...");
          WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
          WS_DEBUG_PRINTLN("Published!");

          // set the pin value in the digital pin object for comparison on next
          // run
          _digital_input_pins[i].prvPinVal = pinVal;

          // reset the digital pin
          _digital_input_pins[i].prvPeriod = curTime;
        }
      }
    }
  }
}